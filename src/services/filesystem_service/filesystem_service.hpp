#ifndef FILESYSTEM_SERVICE_HPP
#define FILESYSTEM_SERVICE_HPP

#include <ch.h>

#include <FilesystemServiceBase.hpp>
#include <filesystem/file.hpp>
#include <xbot-service/Lock.hpp>

class FilesystemService : public FilesystemServiceBase {
 public:
  explicit FilesystemService(uint16_t service_id) : FilesystemServiceBase(service_id, wa, sizeof(wa)) {
  }

 protected:
  bool OnStart() override;

  void RPCListFiles(uint16_t call_id, const char* Path, uint32_t PathLen, char* data,
                    uint16_t* response_length) override;
  void RPCRemoveFile(uint16_t call_id, const char* Path, uint32_t PathLen) override;
  void RPCAddFileChunk(uint16_t call_id, const char* Path, uint32_t PathLen, uint32_t Offset, uint8_t Flags,
                       const uint8_t* Data, uint32_t DataLen) override;

 private:
  // All paths are sandboxed below this root. Path parameters coming in over RPC are relative to it.
  static constexpr const char* kRoot = "/user";

  // Validates rel (rejects "..", a leading '/' or '\', and non-printable characters) and
  // composes out = kRoot + "/" + rel (or just kRoot if rel_len == 0). Returns false on
  // validation failure or if out_size is too small.
  static bool BuildSafePath(const char* rel, uint32_t rel_len, char* out, size_t out_size);

  // littlefs is not inherently thread-safe, and this is the first RPC-driven service to touch
  // it from its own service thread while, e.g., config code elsewhere writes to /cfg/* from
  // other threads. Those existing writes aren't coordinated with any shared lock today, so we
  // can only guard the operations performed by this service.
  MUTEX_DECL(fs_mtx_);

  // Single in-flight upload session, driven by RPCAddFileChunk. Calls are serialized by the
  // service's single processing thread, so this needs no locking against itself.
  File upload_file_;
  char upload_path_[160]{};
  uint32_t upload_next_offset_ = 0;
  bool upload_active_ = false;

  THD_WORKING_AREA(wa, 4096){};
};

#endif  // FILESYSTEM_SERVICE_HPP
