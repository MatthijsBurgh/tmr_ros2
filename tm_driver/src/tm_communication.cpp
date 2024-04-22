#include "tm_driver/tm_communication.h"
#include "tm_driver/tm_driver_utilities.h"
#include "tm_driver/tm_packet.h"
#include "tm_driver/tm_print.h"

#ifdef _WIN32
// windows socket
#pragma comment(lib, "Ws2_32.lib")
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <asm-generic/socket.h>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <cerrno>
#include <sys/socket.h>
#include <sys/select.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#endif

#include <bits/types/struct_timeval.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <thread>
#include <string>
#include <vector>

//
// TmSBuffer
//

TmSBuffer::TmSBuffer()
{
  print_debug("TmSBuffer::TmSBuffer");
}

TmSBuffer::~TmSBuffer()
{
  print_debug("TmSBuffer::~TmSBuffer");
}

int TmSBuffer::append(const char* bdata, int blen)
{
  if (blen <= 0)
    return 0;

  bytes_.reserve(bytes_.size() + blen);
  bytes_.insert(bytes_.end(), bdata, bdata + blen);
  // print_debug("TmSBuffer::append %d bytes", blen);
  return blen;
}

void TmSBuffer::pop_front(int len)
{
  // commit extract
  if (len <= 0)
    return;

  if (size_t(len) < bytes_.size())
  {
    std::vector<decltype(bytes_)::value_type>(bytes_.begin() + len, bytes_.end()).swap(bytes_);
    // std::vector<char> tmp{bytes_.begin() + len, bytes_.end()};
    // bytes_.clear();
    // bytes_.insert(bytes_.end(), tmp.begin(), tmp.end());
  }
  else
  {
    // len = int(bytes_.size());
    bytes_.clear();
  }
  // print_debug("TmSBuffer::pop_front %d bytes", len);
}

//
// TmSvrCommRecv
//

size_t recv_fake_svr_pack_data(char* buf)
{
  static long long cnt = 0;
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  const std::array<float, 6> angle = {0.0f, 0.0f, 90.0f, 0.0f, 90.0f, 0.0f};
  const std::array<float, 6> pose = {420.0f, -120.0f, 360.0f, 180.0f, 0.0f, 90.0f};
  FakeTmSvrPacket svr_pack;
  FakeTmSvrPacket::build_content(svr_pack.content, angle, pose);
  TmSvrData::build_TmSvrData(svr_pack.data, "0", TmSvrData::Mode::BINARY, svr_pack.content.data(),
                             svr_pack.content.size(), TmSvrData::SrcType::Shallow);
  TmSvrData::build_bytes(svr_pack.packet.data, svr_pack.data);
  svr_pack.packet.setup_header(TmPacket::Header::TMSVR);
  std::vector<char> pack_byte;
  TmPacket::build_bytes(pack_byte, svr_pack.packet);
  size_t n = pack_byte.size();
  for (size_t i = 0; i < n; ++i)
  {
    buf[i] = pack_byte[i];
  }
  if (cnt % 10 == 1)
  {
    for (size_t j = 1; j < 7; ++j)
    {
      for (size_t i = 0; i < n; ++i)
      {
        buf[j * n + i] = pack_byte[i];
      }
    }
    n *= 7;
  }
  ++cnt;
  return n;
}

TmCommRecv::TmCommRecv(int recv_buf_len) : recv_buf_(nullptr), recv_buf_len_(std::max(recv_buf_len, 512))
{
  print_debug("TmCommRecv::TmCommRecv");

  recv_buf_ = new char[recv_buf_len_];
  memset(recv_buf_, 0, recv_buf_len_);
}

TmCommRecv::~TmCommRecv()
{
  print_debug("TmCommRecv::~TmCommRecv");
  delete recv_buf_;
}

bool TmCommRecv::setup(int sockfd)
{
  if (sockfd <= 0)
    return false;

  sbuf_.clear();
  sockfd_ = sockfd;

  FD_ZERO(&masterfs_);
  // fake
  if (sockfd != 6188)
  {
    FD_SET(sockfd, &masterfs_);
  }
  rc_ = TmCommRC::OK;
  return true;
}

TmCommRC TmCommRecv::spin_once(int timeval_ms, int* n)
{
  TmCommRC rc = TmCommRC::OK;
  int nb = 0;
  int rv = 0;
  timeval tv;

  // fake
  if (sockfd_ == 6188)
  {
    nb = recv_fake_svr_pack_data(recv_buf_);
    // sbuf_.insert(sbuf_.end(), recv_buf_, recv_buf_ + std::max(nb, 0));
    sbuf_.append(recv_buf_, nb);

    if (n)
      *n = nb;
    rn_ = nb;
    rc_ = rc;
    return rc;
  }

  if (timeval_ms < 8)
    timeval_ms = 8;

  tv.tv_sec = (timeval_ms / 1000);
  tv.tv_usec = (timeval_ms % 1000) * 1000;

  readfs_ = masterfs_;  // re-init

  rv = select(sockfd_ + 1, &readfs_, nullptr, nullptr, &tv);

  if (n)
    *n = 0;

  if (rv < 0)
  {
    rc = TmCommRC::ERR;
  }
  else if (rv == 0)
  {
    rc = TmCommRC::TIMEOUT;
  }
  else if (FD_ISSET(sockfd_, &readfs_))
  {
    nb = recv(sockfd_, recv_buf_, recv_buf_len_, 0);

    if (nb < 0)
    {
      // error
      rc = TmCommRC::ERR;
    }
    else if (nb == 0)
    {
      // sever is closed
      rc = TmCommRC::NOTCONNECT;
    }
    else
    {
      // recv n bytes
      // sbuf_.insert(sbuf_.end(), recv_buf_, recv_buf_ + std::max(nb, 0));
      sbuf_.append(recv_buf_, nb);

      if (n)
        *n = nb;
    }
  }
  else
  {
    rc = TmCommRC::NOTREADY;
  }
  rn_ = nb;
  rc_ = rc;
  return rc;
}

//
// TmCommunication
//

TmCommunication::TmCommunication(const std::string& ip, unsigned short port, int recv_buf_len)
  : recv_(nullptr)
  , ip_(nullptr)
  , port_(port)
  , recv_buf_len_(recv_buf_len)
  , sockfd_(-1)
  , is_connected_(false)
  , opt_flag_(1)
  , recv_rc_(TmCommRC::OK)
  , recv_ready_(false)
{
  print_debug("TmCommunication::TmCommunication");

  recv_ = std::make_unique<TmCommRecv>(recv_buf_len_);

  ip_ = new char[ip.size() + 1];
  std::copy(ip.begin(), ip.end(), ip_);
  ip_[ip.size()] = '\0';

#ifdef _WIN32
  // Initialize Winsock
  WSADATA wsaData;
  int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
  if (iResult != 0)
  {
    //
  }
#endif
}

TmCommunication::~TmCommunication()
{
  print_debug("TmCommunication::~TmCommunication");

  if (ip_)
  {
    delete ip_;
    ip_ = nullptr;
  }

#ifdef _WIN32
  // cleanup
  WSACleanup();
#endif
}

uint64_t TmCommunication::get_current_time_in_ms()
{
  const std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
  const std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());
  return ms.count();
}

int TmCommunication::connect_with_timeout(int sockfd, const char* ip, unsigned short port, int timeout_ms)
{
  int rv = 0;
  int flags = 0;
  int err = 0;
  int err_len = 0;
  sockaddr_in addr;
  timeval tv;
  fd_set wset;

  print_once("TM_COM: ip:=%s", ip);

  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  inet_pton(AF_INET, ip, &(addr.sin_addr));

  tv.tv_sec = (timeout_ms / 1000);
  tv.tv_usec = (timeout_ms % 1000) * 1000;

  FD_ZERO(&wset);
  FD_SET(sockfd, &wset);

#ifndef _WIN32
  // Get Flag of Fcntl
  if ((flags = fcntl(sockfd, F_GETFL, 0)) < 0)
  {
    print_warn("TM_COM: The flag of fcntl is not ok");
    return -1;
  }
#endif

  rv = connect(sockfd, (sockaddr*)&addr, 16);
  print_debug("TM_COM: rv:=%d", (int)rv);

  if (rv < 0)
  {
    if (errno != EINPROGRESS)
      return -1;
  }
  if (rv == 0)
  {
    timeout_count_ = 0;
    print_debug("TM_COM: Connection is ok");
    return rv;
  }
  else
  {
    timeout_count_++;
    // Wait for Connect OK by checking Write buffer
    if ((rv = select(sockfd + 1, nullptr, &wset, nullptr, &tv)) < 0)
    {
      return rv;
    }
    if (rv == 0)
    {
      print_warn("TM_COM: Connection timeout count: %d", timeout_count_);
      // errno = ETIMEDOUT;
      return -1;
    }
    if (FD_ISSET(sockfd, &wset))
    {
#ifdef _WIN32
      if (getsockopt(sockfd, SOL_SOCKET, SO_ERROR, (char*)&err, &err_len) < 0)
      {
#else
      if (getsockopt(sockfd, SOL_SOCKET, SO_ERROR, &err, (socklen_t*)&err_len) < 0)
      {
#endif
        print_error("TM_COM: Get socketopt SO_ERROR FAIL");
        errno = err;
        return -1;
      }
    }
    else
    {
      print_error("TM_COM: Connection is not ready");
      return -1;
    }
    if (err != 0)
    {
      errno = err;
      print_error("TM_COM: Connection error");
      return -1;
    }
  }
  return rv;
}

bool TmCommunication::connect_socket(const std::string& error_name, int timeout_ms)
{
  is_connected_ = false;
  if (sockfd_ > 0)
    return true;

  if (timeout_ms < 0)
    timeout_ms = 0;

#ifdef _WIN32
  addrinfo hints;
  ZeroMemory(&hints, sizeof(hints));
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_protocol = IPPROTO_HOPOPTS;

  socket_file_ = socket(hints.ai_family, hints.ai_socktype, hints.ai_protocol);
#else
  socket_file_ = socket(AF_INET, SOCK_STREAM, 0);
#endif
  sockfd_ = socket_file_;
  if (sockfd_ < 0)
  {
    const std::string error_msg = "TM_COM (" + error_name + "): Error socket";
    print_error(error_msg.c_str());
    return false;
  }

  setsockopt(sockfd_, IPPROTO_TCP, TCP_NODELAY, (char*)&opt_flag_, sizeof(opt_flag_));
#ifndef _WIN32
  setsockopt(sockfd_, IPPROTO_TCP, TCP_QUICKACK, (char*)&opt_flag_, sizeof(opt_flag_));
#endif
  setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (char*)&opt_flag_, sizeof(opt_flag_));
  struct timeval timeout;
  timeout.tv_sec = timeout_ms / 1000;
  timeout.tv_usec = 0;

  if (setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout)) < 0)
  {
    const std::string error_msg = error_name + "setsockopt failed\n";
    print_error(error_msg.c_str());
  }

  if (setsockopt(sockfd_, SOL_SOCKET, SO_SNDTIMEO, (char*)&timeout, sizeof(timeout)) < 0)
  {
    const std::string error_msg = error_name + "setsockopt failed\n";
    print_error(error_msg.c_str());
  }

  if (connect_with_timeout(sockfd_, ip_, port_, timeout_ms) == 0)
  {
    const std::string error_msg = "TM_COM (" + error_name + "): O_NONBLOCK connection is ok";
    print_debug(error_msg.c_str());
    is_connected_ = true;
  }
  else
  {
    const std::string error_msg = "TM_COM (" + error_name + "): O_NONBLOCK connection is fail";
    print_debug(error_msg.c_str());
    sockfd_ = -1;
    is_connected_ = false;
  }
  if (sockfd_ > 0)
  {
    const std::string msg = "TM_COM (" + error_name + "): TM robot is connected. sockfd:=" + std::to_string(sockfd_);
    print_info(msg.c_str());
    //_is_connected = true;
    return true;
  }
  else
  {
    return false;
  }
}

void TmCommunication::close_socket()
{
  is_connected_ = false;
  // reset
  recv_rc_ = TmCommRC::OK;
  recv_ready_ = false;

#ifdef _WIN32
  closesocket((SOCKET)socket_file_);
#else
  close(socket_file_);
#endif
  sockfd_ = -1;
}

TmCommRC TmCommunication::send_bytes(const char* bytes, int len, int* n)
{
  TmCommRC rc = TmCommRC::OK;

  if (n)
    *n = 0;

  if (len <= 0)
    return TmCommRC::OK;
  if (sockfd_ < 0)
    return TmCommRC::NOTREADY;

  const int nb = send(sockfd_, bytes, len, 0);

  if (nb < 0)
  {
    rc = TmCommRC::ERR;
  }
  else if (nb < len)
  {
    rc = TmCommRC::NOTSENDALL;

    if (n)
      *n = nb;
  }
  return rc;
}

TmCommRC TmCommunication::send_bytes_all(const char* bytes, int len, int* n)
{
  TmCommRC rc = TmCommRC::OK;

  if (n)
    *n = 0;

  if (len <= 0)
    return TmCommRC::OK;
  if (sockfd_ < 0)
    return TmCommRC::NOTREADY;

  int ntotal = 0;
  int nb = 0;
  int nleft = len;

  while (ntotal < len)
  {
    nb = send(sockfd_, bytes + ntotal, nleft, 0);
    if (nb < 0)
    {
      rc = TmCommRC::ERR;
      break;
    }
    ntotal += nb;
    nleft -= nb;
  }
  if (n)
    *n = ntotal;
  return rc;
}

TmCommRC TmCommunication::send_packet(TmPacket& packet, int* n)
{
  std::vector<char> bytes;
  TmPacket::build_bytes(bytes, packet);
  print_info(TmPacket::string_from_bytes(bytes).c_str());
  return send_bytes(bytes.data(), bytes.size(), n);
}

TmCommRC TmCommunication::send_packet_all(TmPacket& packet, int* n)
{
  std::vector<char> bytes;
  TmPacket::build_bytes(bytes, packet);
  print_info(TmPacket::string_from_bytes(bytes).c_str());
  return send_bytes_all(bytes.data(), bytes.size(), n);
}

TmCommRC TmCommunication::send_packet_(TmPacket& packet, int* n)
{
  std::vector<char> bytes;
  TmPacket::build_bytes(bytes, packet);
  print_info(TmPacket::string_from_bytes(bytes).c_str());
  if (bytes.size() > 0x1000)
    return send_bytes_all(bytes.data(), bytes.size(), n);

  return send_bytes(bytes.data(), bytes.size(), n);
}

TmCommRC TmCommunication::send_packet_silent(TmPacket& packet, int* n)
{
  std::vector<char> bytes;
  TmPacket::build_bytes(bytes, packet);
  return send_bytes(bytes.data(), bytes.size(), n);
}

TmCommRC TmCommunication::send_packet_silent_all(TmPacket& packet, int* n)
{
  std::vector<char> bytes;
  TmPacket::build_bytes(bytes, packet);
  return send_bytes_all(bytes.data(), bytes.size(), n);
}

TmCommRC TmCommunication::send_packet_silent_(TmPacket& packet, int* n)
{
  std::vector<char> bytes;
  TmPacket::build_bytes(bytes, packet);
  if (bytes.size() > 0x1000)
    return send_bytes_all(bytes.data(), bytes.size(), n);

  return send_bytes(bytes.data(), bytes.size(), n);
}

bool TmCommunication::recv_init()
{
  recv_ready_ = recv_->setup(sockfd_);
  return recv_ready_;
}

TmCommRC TmCommunication::recv_spin_once(int timeval_ms, int* n)
{
  TmCommRC rc = TmCommRC::OK;

  if (n)
    *n = 0;

  // if (sockfd_ <= 0) return TmCommRC::NOTCONNECT;

  // first init.
  /*if (!recv_ready_) {
    if (recv_->setup(sockfd_))
      recv_ready_ = true;
    else
      return TmCommRC::NOTREADY;
  }*/

  // spin once
  int nb = 0;
  rc = recv_->spin_once(timeval_ms, &nb);

  if (n)
    *n = nb;

  // error handling
  if (rc != TmCommRC::OK)
  {
    recv_rc_ = rc;
    return rc;
  }

  // find packet
  int loop_cnt = 0;
  int pack_cnt = 0;
  int blen = 0;
  size_t size = 0;
  size_t len = 0;
  bool ncs = false;
  bool ok = false;

  while (loop_cnt < 10 || pack_cnt < 10)
  {
    // blen = recv_->buffer().size();
    blen = recv_->buffer().length();
    if (blen < 9)
    {
      break;
    }

    // print_debug("TmCommunication::recv_spin_once: %d, %d", bdata, loop_cnt);

    ++size;
    packet_list_.resize(size);

    // len = TmPacket::build_packet_from_bytes(packet_list_.back(), &recv_->buffer().front(), blen);
    len = TmPacket::build_packet_from_bytes(packet_list_.back(), recv_->buffer().data(), blen);

    ncs = packet().is_checksum_failed();
    ok = packet().is_valid();

    if (ok || ncs)
    {
      // auto& buffer = recv_->buffer();
      // buffer.erase(buffer.begin(), buffer.begin() + std::max<size_t>(len, 0));
      recv_->buffer().pop_front(len);
    }
    if (ok)
    {
      ++pack_cnt;
    }
    else
    {
      if (size > 1)
      {
        packet_list_.resize(size - 1);
      }
      // if (pack_cnt != 0) break;
      break;
    }
    ++loop_cnt;
  }
  if (loop_cnt == 10 || pack_cnt == 10)
  {
    print_warn("sticky bag over 10 packages, to recevie data more fluently, please check your net!");
  }
  if (pack_cnt == 0)
  {
    rc = TmCommRC::NOVALIDPACK;
  }
  recv_rc_ = rc;
  return rc;
}
