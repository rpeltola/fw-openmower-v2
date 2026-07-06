#include "filesystem_service.hpp"

#include <ulog.h>

#include <cstdio>
#include <cstring>
#include <filesystem/filesystem.hpp>

using xbot::datatypes::RpcStatus;

bool FilesystemService::OnStart() {
  xbot::service::Lock lk{&fs_mtx_};
  int result = lfs_mkdir(&lfs, kRoot);
  if (result != LFS_ERR_OK && result != LFS_ERR_EXIST) {
    ULOG_ARG_ERROR(&service_id_, "Failed to create %s: error=%d", kRoot, result);
    return false;
  }
  return true;
}

bool FilesystemService::BuildSafePath(const char* rel, uint32_t rel_len, char* out, size_t out_size) {
  if (rel == nullptr) {
    return false;
  }

  for (uint32_t i = 0; i < rel_len; i++) {
    char c = rel[i];
    if (c < 0x20 || c > 0x7E) {
      return false;  // non-printable
    }
  }
  if (rel_len > 0 && (rel[0] == '/' || rel[0] == '\\')) {
    return false;
  }
  for (uint32_t i = 0; i + 1 < rel_len; i++) {
    if (rel[i] == '.' && rel[i + 1] == '.') {
      return false;  // reject any path traversal attempt
    }
  }

  const size_t root_len = strlen(kRoot);
  if (rel_len == 0) {
    if (root_len + 1 > out_size) {
      return false;
    }
    memcpy(out, kRoot, root_len);
    out[root_len] = '\0';
    return true;
  }

  const size_t needed = root_len + 1 + rel_len + 1;  // kRoot + '/' + rel + NUL
  if (needed > out_size) {
    return false;
  }

  size_t idx = 0;
  memcpy(out + idx, kRoot, root_len);
  idx += root_len;
  out[idx++] = '/';
  memcpy(out + idx, rel, rel_len);
  idx += rel_len;
  out[idx] = '\0';
  return true;
}

void FilesystemService::RPCListFiles(uint16_t call_id, const char* Path, uint32_t PathLen, char* data,
                                     uint16_t* response_length) {
  char dir_path[160];
  if (!BuildSafePath(Path, PathLen, dir_path, sizeof(dir_path))) {
    SendRpcResponse(call_id, RpcStatus::ERROR, nullptr, 0);
    return;
  }

  xbot::service::Lock lk{&fs_mtx_};

  lfs_dir_t dir;
  if (lfs_dir_open(&lfs, &dir, dir_path) != LFS_ERR_OK) {
    SendRpcResponse(call_id, RpcStatus::ERROR, nullptr, 0);
    return;
  }

  const uint16_t capacity = *response_length;
  uint16_t written = 0;
  struct lfs_info info {};
  while (lfs_dir_read(&lfs, &dir, &info) > 0) {
    if (strcmp(info.name, ".") == 0 || strcmp(info.name, "..") == 0) {
      continue;
    }
    char line[LFS_NAME_MAX + 1 + 16];
    int line_len = snprintf(line, sizeof(line), "%s\t%lu\n", info.name, static_cast<unsigned long>(info.size));
    if (line_len <= 0) {
      continue;
    }
    if (static_cast<uint32_t>(written) + static_cast<uint32_t>(line_len) > capacity) {
      break;  // next line would overflow the response buffer, stop cleanly
    }
    memcpy(data + written, line, line_len);
    written += static_cast<uint16_t>(line_len);
  }
  lfs_dir_close(&lfs, &dir);

  *response_length = written;
  SendRpcResponse(call_id, RpcStatus::SUCCESS, data, *response_length);
}

void FilesystemService::RPCRemoveFile(uint16_t call_id, const char* Path, uint32_t PathLen) {
  FsResult result;
  char path[160];

  if (PathLen == 0 || !BuildSafePath(Path, PathLen, path, sizeof(path))) {
    result = FsResult::ERR_PATH;
  } else {
    xbot::service::Lock lk{&fs_mtx_};
    int res = lfs_remove(&lfs, path);
    if (res == LFS_ERR_OK) {
      result = FsResult::OK;
    } else if (res == LFS_ERR_NOENT) {
      result = FsResult::ERR_NOENT;
    } else {
      result = FsResult::ERR_IO;
    }
  }

  uint8_t r = static_cast<uint8_t>(result);
  SendRpcResponse(call_id, RpcStatus::SUCCESS, &r, 1);
}

void FilesystemService::RPCAddFileChunk(uint16_t call_id, const char* Path, uint32_t PathLen, uint32_t Offset,
                                        uint8_t Flags, const uint8_t* Data, uint32_t DataLen) {
  FsResult result = FsResult::ERR_PATH;
  char path[160];

  if (PathLen != 0 && BuildSafePath(Path, PathLen, path, sizeof(path))) {
    xbot::service::Lock lk{&fs_mtx_};
    result = FsResult::OK;

    if ((Flags & FsChunkFlags::FIRST) && Offset != 0) {
      // A FIRST chunk always starts at offset 0; reject before touching the target file
      // so a malformed request can't truncate an existing file.
      result = FsResult::ERR_SESSION;
    } else if (Flags & FsChunkFlags::FIRST) {
      // File::open() fails if already open, so any in-flight upload must be closed first,
      // regardless of whether it targeted the same path.
      if (upload_active_) {
        upload_file_.close();
        upload_active_ = false;
      }
      if (upload_file_.mkdirp(path) != LFS_ERR_OK) {
        result = FsResult::ERR_IO;
      } else if (upload_file_.open(path, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC) != LFS_ERR_OK) {
        result = FsResult::ERR_IO;
      } else {
        strncpy(upload_path_, path, sizeof(upload_path_) - 1);
        upload_path_[sizeof(upload_path_) - 1] = '\0';
        upload_next_offset_ = 0;
        upload_active_ = true;
      }
    }

    if (result == FsResult::OK) {
      if (!upload_active_ || strcmp(path, upload_path_) != 0 || Offset != upload_next_offset_) {
        result = FsResult::ERR_SESSION;
      } else {
        int written = upload_file_.write(const_cast<uint8_t*>(Data), DataLen);
        if (written < 0) {
          result = (written == LFS_ERR_NOSPC) ? FsResult::ERR_NOSPC : FsResult::ERR_IO;
        } else {
          upload_next_offset_ += static_cast<uint32_t>(written);
          if (Flags & FsChunkFlags::LAST) {
            upload_file_.sync();
            upload_file_.close();
            upload_active_ = false;
          }
        }
      }
    }
  }

  uint8_t r = static_cast<uint8_t>(result);
  SendRpcResponse(call_id, RpcStatus::SUCCESS, &r, 1);
}
