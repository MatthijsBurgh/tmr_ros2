#include "tm_driver.h"

class ListenNodeConnection
{
private:
  std::thread listenNodeThread;
  std::thread checkListenNodeThread;
  TmDriver& iface;
  TmSctCommunication& sct_;
  std::function<void(TmSctData)> sct_msg;
  std::function<void(std::string, std::string)> sta_msg;

  std::mutex checkIsOnListenNodeMutex;
  std::condition_variable checkIsOnListenNodeCondVar;
  std::mutex firstCheckIsOnListenNodeMutex;
  std::condition_variable firstCheckIsOnListenNodeCondVar;
  std::mutex sta_mtx_;
  bool sta_updated_;
  std::condition_variable sta_cv_;

  int sct_reconnect_timeout_ms_ = 1000;
  int sct_reconnect_timeval_ms_ = 3000;
  bool firstEnter = true;
  std::string staSubcmd;
  std::string staSubdata;
  bool isRun;

  void listen_node_connect();
  void sct_connect_recover();
  bool send_data();
  void check_is_on_listen_node();
  void build_sta_cmd();

public:
  ListenNodeConnection(TmDriver& iface, const std::function<void(TmSctData)>& sct_msg,
                       const std::function<void(std::string, std::string)>& sta_msg, bool is_fake_);
  bool connect_tmsct(int timeout, int timeval, bool connect, bool reconnect);
  bool send_listen_node_script(const std::string& id, const std::string& script);
  bool ask_sta_struct(const std::string& subcmd, const std::string& subdata, double waitTime, std::string& reSubcmd,
                      std::string& reSubdata);
  void check_is_on_listen_node_from_script(const std::string& id, const std::string& script);
  ~ListenNodeConnection();
};
