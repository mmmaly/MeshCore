#pragma once
// A FILESYSTEM/File shim over stdio, so IdentityStore and DataStore run on a
// host. Mirrors the Arduino FS API surface those two actually use: open with
// mode strings, exists/remove/mkdir/format/info, and a File with
// read/write/seek/close plus a bool test.
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>
#include "Stream.h"   // Arduino File is a Stream; IdentityStore relies on it
#include <errno.h>

#ifndef FILE_O_READ
#define FILE_O_READ  "r"
#define FILE_O_WRITE "w"
#endif

struct FSInfo { size_t totalBytes = 0; size_t usedBytes = 0; };

class File : public Stream {
  FILE* _f = nullptr;
  std::string _path;
public:
  File() {}
  File(FILE* f, std::string path) : _f(f), _path(std::move(path)) {}
  operator bool() const { return _f != nullptr || _dir != nullptr; }
  bool operator!() const { return _f == nullptr && _dir == nullptr; }

  int read(uint8_t* buf, size_t len) { return _f ? (int)fread(buf, 1, len, _f) : 0; }
  int read() override { if (!_f) return -1; return fgetc(_f); }
  size_t readBytes(char* buf, size_t len) override { return _f ? fread(buf, 1, len, _f) : 0; }
  size_t readBytes(uint8_t* buf, size_t len) override { return _f ? fread(buf, 1, len, _f) : 0; }
  size_t write(const uint8_t* buf, size_t len) override { return _f ? fwrite(buf, 1, len, _f) : 0; }
  size_t write(uint8_t b) override { return write(&b, 1); }
  using Stream::write;
  bool seek(uint32_t pos) { return _f && fseek(_f, (long)pos, SEEK_SET) == 0; }
  uint32_t position() { return _f ? (uint32_t)ftell(_f) : 0; }
  uint32_t size() {
    if (!_f) return 0;
    long cur = ftell(_f); fseek(_f, 0, SEEK_END);
    long end = ftell(_f); fseek(_f, cur, SEEK_SET);
    return (uint32_t)end;
  }
  int available() override { return _f ? (int)(size() - position()) : 0; }
  void flush() override { if (_f) fflush(_f); }
  void close() { if (_f) { fclose(_f); _f = nullptr; } if (_dir) { closedir(_dir); _dir = nullptr; } }

  // Directory iteration: the companion lists stored blobs this way.
  bool isDirectory() { return _dir != nullptr; }
  const char* name() { return _name.c_str(); }
  File openNextFile() {
    if (!_dir) return File();
    struct dirent* e;
    while ((e = readdir(_dir)) != nullptr) {
      if (!strcmp(e->d_name, ".") || !strcmp(e->d_name, "..")) continue;
      std::string child = _path + "/" + e->d_name;
      struct stat st;
      if (stat(child.c_str(), &st) == 0 && S_ISDIR(st.st_mode)) {
        File d; d._dir = opendir(child.c_str()); d._path = child; d._name = e->d_name;
        return d;
      }
      File r(fopen(child.c_str(), "rb"), child);
      r._name = e->d_name;
      return r;
    }
    return File();
  }
  friend class HostFS;
private:
  DIR* _dir = nullptr;
  std::string _name;
};

class HostFS {
  std::string _root;
  std::string full(const char* p) const {
    std::string s(p);
    if (!s.empty() && s[0] == '/') s.erase(0, 1);
    return _root + "/" + s;
  }
public:
  explicit HostFS(const char* root = ".") : _root(root) { ::mkdir(_root.c_str(), 0700); }

  File open(const char* path, const char* mode = "r", bool create = false) {
    std::string p = full(path);
    struct stat st;
    if (stat(p.c_str(), &st) == 0 && S_ISDIR(st.st_mode)) {
      File d; d._dir = opendir(p.c_str()); d._path = p; d._name = path;
      return d;                       // opening a directory = iterate it
    }
    // Writing to a path whose directory does not exist is normal on the
    // flat SPIFFS namespace firmware usually runs on; here it needs a mkdir.
    if (mode && (mode[0] == 'w' || mode[0] == 'a')) {
      size_t slash = p.find_last_of('/');
      if (slash != std::string::npos) ::mkdir(p.substr(0, slash).c_str(), 0700);
    }
    FILE* f = fopen(p.c_str(), mode);
    if (!f && create) f = fopen(p.c_str(), "w+");
    File r(f, p);
    r._name = path;
    return r;
  }
  bool exists(const char* path) { return access(full(path).c_str(), F_OK) == 0; }
  bool remove(const char* path) { return ::remove(full(path).c_str()) == 0; }
  bool mkdir(const char* path) { return ::mkdir(full(path).c_str(), 0700) == 0 || errno == EEXIST; }
  bool format() { return true; }        // never wipe a host filesystem
  bool info(FSInfo& i) { i.totalBytes = 0; i.usedBytes = 0; return true; }
};

#define FILESYSTEM HostFS
