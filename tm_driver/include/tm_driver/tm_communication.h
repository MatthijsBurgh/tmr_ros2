#pragma once
#include "tm_packet.h"
#include "tm_driver_utilities.h"

class TmCommRecv;

class TmCommunication
{
private:
  std::unique_ptr<TmCommRecv> recv_;
  char* ip_;
  unsigned short port_;
  int recv_buf_len_;
  int sockfd_;
  mutable bool is_connected_;
  int socket_file_;
  int opt_flag_;
  TmCommRC recv_rc_;
  bool recv_ready_;

  int timeout_count_ = 0;

  std::vector<TmPacket> packet_list_;

public:
  explicit TmCommunication(const std::string& ip, unsigned short port, int recv_buf_len);
  virtual ~TmCommunication();

  int socket_description() const
  {
    return sockfd_;
  }

  int socket_description(int sockfd)
  {
    sockfd_ = sockfd;
    return sockfd_;
  }

  bool is_connected() const
  {
    if (sockfd_ < 0)
    {
      is_connected_ = false;
    }
    return is_connected_;
  }

  bool connect_socket(const std::string& error_name, int timeout_ms = 0);

  void close_socket();

  TmCommRC send_bytes(const char* bytes, int len, int* n = nullptr);

  TmCommRC send_bytes_all(const char* bytes, int len, int* n = nullptr);

  TmCommRC send_packet(TmPacket& packet, int* n = nullptr);

  TmCommRC send_packet_all(TmPacket& packet, int* n = nullptr);

  TmCommRC send_packet_(TmPacket& packet, int* n = nullptr);

  TmCommRC send_packet_silent(TmPacket& packet, int* n = nullptr);

  TmCommRC send_packet_silent_all(TmPacket& packet, int* n = nullptr);

  TmCommRC send_packet_silent_(TmPacket& packet, int* n = nullptr);

  bool recv_init();

  TmCommRC recv_spin_once(int timeval_ms, int* n = nullptr);

  inline TmCommRC recv_rc()
  {
    return recv_rc_;
  }

  std::vector<TmPacket>& packet_list()
  {
    return packet_list_;
  }

  TmPacket& packet()
  {
    return packet_list_.back();
  }

  static uint64_t get_current_time_in_ms();

private:
  int connect_with_timeout(int sockfd, const char* ip, unsigned short port, int timeout_ms);
};

class TmSBuffer
{
private:
  std::vector<char> bytes_;

public:
  TmSBuffer();

  ~TmSBuffer();

  inline int length() const
  {
    return bytes_.size();
  }

  inline char* data()
  {
    return bytes_.data();
  }

  int append(const char* bdata, int blen);

  void pop_front(int len = 1);

  inline void clear()
  {
    bytes_.clear();
  }
};

class TmCommRecv
{
private:
  TmSBuffer sbuf_;
  char* recv_buf_;
  int recv_buf_len_ = 0;
  int sockfd_ = -1;
  fd_set masterfs_;
  fd_set readfs_;
  int rn_ = 0;
  TmCommRC rc_ = TmCommRC::OK;

public:
  explicit TmCommRecv(int recv_buf_len);

  ~TmCommRecv();

  bool setup(int sockfd);

  TmCommRC spin_once(int timeval_ms, int* n = nullptr);

  inline void commit_spin_once()
  {
    sbuf_.pop_front(rn_);
  };

  inline TmSBuffer& buffer()
  {
    return sbuf_;
  }
};
