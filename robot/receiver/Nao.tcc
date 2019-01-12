#include "utils/Logger.hpp"
#include <boost/asio/placeholders.hpp>

template <class SubClass, typename ReadHandler>
NaoReceiver::NaoReceiver(SubClass *scthis, ReadHandler handler, int port)
   : service(), socket(service, boost::asio::ip::udp::v4()) {

   llog(INFO) << "Nao Receiver constructed" << std::endl;
   // Allows multiple processes to listen on the same port (so we can run
   // multiple instances of rUNSWift for simulation)
   boost::asio::socket_base::reuse_address option(true);
   socket.set_option(option);  

   socket.bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port));

   startReceive(scthis, handler);
   t = new boost::thread(boost::bind(&boost::asio::io_service::run, &service));
   llog(INFO) << "Listening for data on port " << port << std::endl;
}

template <class SubClass, typename ReadHandler>
void NaoReceiver::startReceive(SubClass *scthis, ReadHandler handler) {
   socket.async_receive_from(
      boost::asio::buffer((boost::asio::mutable_buffer((void *)&recvBuffer,
                  sizeof(recvBuffer)))), remoteEndpoint,   
                  boost::bind(handler, scthis,
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));
}
