#pragma once

#include <cstddef>
#include <map>
#include <string>
#include <vector>
#include <utility>
#include <time.h>

#include <capnp/serialize.h>

#include "cereal/gen/cpp/log.capnp.h"
// Use CLOCK_MONOTONIC if CLOCK_BOOTTIME is not available (e.g., on macOS)
#ifdef __APPLE__
#define CLOCK_BOOTTIME CLOCK_MONOTONIC
#endif

#define MSG_MULTIPLE_PUBLISHERS 100
// Function to check if ZeroMQ is used for messaging
bool messaging_use_zmq();
// Abstract class for a messaging context (possibly wrapping a ZeroMQ context)
class Context {
public:
  virtual void * getRawContext() = 0; // Get the underlying messaging context
  static Context * create(); // Factory method to create a Context instance
  virtual ~Context(){}
};
// Abstract class for a message
class Message {
public:
  virtual void init(size_t size) = 0; // Initialize a message with a given size
  virtual void init(char * data, size_t size) = 0; // Initialize a message with data and size
  virtual void close() = 0; // Clean up resources used by the message
  virtual size_t getSize() = 0; // Get the size of the message
  virtual char * getData() = 0; // Get the data of the message
  virtual ~Message(){}
};

// Abstract class for a subscription socket
class SubSocket {
public:
  // Connect the subscription socket to a specified endpoint
  virtual int connect(Context *context, std::string endpoint, std::string address, bool conflate=false, bool check_endpoint=true) = 0;
  virtual void setTimeout(int timeout) = 0; // Set a timeout for receiving messages
  virtual Message *receive(bool non_blocking=false) = 0; // Receive a message
  virtual void * getRawSocket() = 0; // Get the underlying socket
  static SubSocket * create(); // Factory method to create a SubSocket instance
  // Convenience method to create and connect a SubSocket
  static SubSocket * create(Context * context, std::string endpoint, std::string address="127.0.0.1", bool conflate=false, bool check_endpoint=true);
  virtual ~SubSocket(){}
};

// Abstract class for a publication socket
class PubSocket {
public:
  // Connect the publication socket to a specified endpoint
  virtual int connect(Context *context, std::string endpoint, bool check_endpoint=true) = 0;
  virtual int sendMessage(Message *message) = 0; // Send a message
  virtual int send(char *data, size_t size) = 0; // Send raw data
  virtual bool all_readers_updated() = 0; // Check if all readers have received the latest message
  static PubSocket * create(); // Factory method to create a PubSocket instance
  // Convenience methods to create and connect a PubSocket
  static PubSocket * create(Context * context, std::string endpoint, bool check_endpoint=true);
  static PubSocket * create(Context * context, std::string endpoint, int port, bool check_endpoint=true);
  virtual ~PubSocket(){}
};

// Abstract class for a poller, which waits for messages on multiple sockets
class Poller {
public:
  virtual void registerSocket(SubSocket *socket) = 0; // Register a socket to poll for messages
  virtual std::vector<SubSocket*> poll(int timeout) = 0; // Wait for messages on registered sockets
  static Poller * create(); // Factory method to create a Poller instance
  static Poller * create(std::vector<SubSocket*> sockets); // Convenience method to create a Poller with sockets
  virtual ~Poller(){}
};
class SubMaster {
public:
  // Constructor: Initializes a SubMaster with a list of services to subscribe to.
  // Optionally specify services to poll, a specific address, and services to ignore in "alive" checks.
  SubMaster(const std::vector<const char *> &service_list, const std::vector<const char *> &poll = {},
            const char *address = nullptr, const std::vector<const char *> &ignore_alive = {});

  // Update subscribed messages with a timeout. This method fetches new messages from all subscribed services.
  void update(int timeout = 1000);

  // Process messages based on current time and a list of received messages. Typically used to update internal state with new message data.
  void update_msgs(uint64_t current_time, const std::vector<std::pair<std::string, cereal::Event::Reader>> &messages);

  // Check if all specified services are alive (i.e., sending messages).
  inline bool allAlive(const std::vector<const char *> &service_list = {}) { return all_(service_list, false, true); }

  // Check if all specified services have valid (i.e., correctly formatted) messages.
  inline bool allValid(const std::vector<const char *> &service_list = {}) { return all_(service_list, true, false); }

  // Check if all specified services are both alive and have valid messages.
  inline bool allAliveAndValid(const std::vector<const char *> &service_list = {}) { return all_(service_list, true, true); }

  // Remove all messages from the subscription queue. Useful for clearing out stale data.
  void drain();

  // Destructor: Cleans up resources used by the SubMaster, such as allocated SubSockets.
  ~SubMaster();

  // Other members:
  uint64_t frame = 0; // Frame counter for tracking updates.

  // Methods to check if a specific service has updated, is alive, is valid, and to get the receive frame and time.
  bool updated(const char *name) const;
  bool alive(const char *name) const;
  bool valid(const char *name) const;
  uint64_t rcv_frame(const char *name) const;
  uint64_t rcv_time(const char *name) const;

  // Operator overloading to access the latest message from a service by its name.
  cereal::Event::Reader &operator[](const char *name) const;

private:
  // Helper function used by allAlive, allValid, and allAliveAndValid methods.
  bool all_(const std::vector<const char *> &service_list, bool valid, bool alive);

  Poller *poller_ = nullptr; // Pointer to a Poller instance for managing socket polling.

  struct SubMessage; // Forward declaration of a private structure to manage subscription messages.
  std::map<SubSocket *, SubMessage *> messages_; // Maps subscription sockets to their messages.
  std::map<std::string, SubMessage *> services_; // Maps service names to their messages.
};

class MessageBuilder : public capnp::MallocMessageBuilder {
public:
  MessageBuilder() = default;

  // Initializes a new `cereal::Event` message, setting its timestamp and validity.
  cereal::Event::Builder initEvent(bool valid = true) {
    cereal::Event::Builder event = initRoot<cereal::Event>();
    struct timespec t;
    clock_gettime(CLOCK_BOOTTIME, &t);
    uint64_t current_time = t.tv_sec * 1000000000ULL + t.tv_nsec;
    event.setLogMonoTime(current_time); // Sets the message's timestamp.
    event.setValid(valid); // Marks the message as valid or invalid.
    return event;
  }

  // Serializes the built message to a byte array for transmission or storage.
  kj::ArrayPtr<capnp::byte> toBytes() {
    heapArray_ = capnp::messageToFlatArray(*this);
    return heapArray_.asBytes();
  }

  // Gets the size of the serialized message in bytes.
  size_t getSerializedSize() {
    return capnp::computeSerializedSizeInWords(*this) * sizeof(capnp::word);
  }

  // Serializes the message into a provided buffer, checking for buffer overflow.
  int serializeToBuffer(unsigned char *buffer, size_t buffer_size) {
    size_t serialized_size = getSerializedSize();
    if (serialized_size > buffer_size) { return -1; }; // Buffer too small.
    kj::ArrayOutputStream out(kj::ArrayPtr<capnp::byte>(buffer, buffer_size));
    capnp::writeMessage(out, *this);
    return serialized_size; // Returns the size of the serialized data.
  }

private:
  kj::Array<capnp::word> heapArray_; // Internal storage for the serialized message.
};

class PubMaster {
public:
  // Constructor initializes sockets for a list of services.
  PubMaster(const std::vector<const char *> &service_list);

  // Sends data as a message to a specified service.
  inline int send(const char *name, capnp::byte *data, size_t size) {
    return sockets_.at(name)->send((char *)data, size);
  }

  // Uses a `MessageBuilder` to build and send a message to a specified service.
  int send(const char *name, MessageBuilder &msg);

  // Destructor cleans up the allocated PubSocket instances.
  ~PubMaster();

private:
  std::map<std::string, PubSocket *> sockets_; // Maps service names to their publication sockets.
};


class AlignedBuffer {
public:
  // Aligns raw data to the nearest word boundary, preparing it for Cap'n Proto operations.
  kj::ArrayPtr<const capnp::word> align(const char *data, const size_t size) {
    words_size = size / sizeof(capnp::word) + 1; // Calculate necessary words.
    if (aligned_buf.size() < words_size) {
      // Allocate more space if the current buffer is too small.
      aligned_buf = kj::heapArray<capnp::word>(words_size < 512 ? 512 : words_size);
    }
    memcpy(aligned_buf.begin(), data, size); // Copy the data into the aligned buffer.
    return aligned_buf.slice(0, words_size); // Return a pointer to the aligned data.
  }

  // Convenience method to align data directly from a `Message` instance.
  inline kj::ArrayPtr<const capnp::word> align(Message *m) {
    return align(m->getData(), m->getSize());
  }

private:
  kj::Array<capnp::word> aligned_buf; // Buffer for aligned data.
  size_t words_size; // The size of the buffer in words.
};
