#ifndef SWRI_ROSCPP_LATCHED_SUBSCRIBER_H_
#define SWRI_ROSCPP_LATCHED_SUBSCRIBER_H_


#include <ros/node_handle.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <swri_roscpp/subscriber.h>

namespace swri
{
// THIS CLASS IS DEPRECATED.  This class has been replaced with a
// secondary interface to Subscriber that is initialized with the
// address of a boost::shared_ptr<M const> variable.  When a message
// is received, it's value is assigned to this pointer.  This approach
// provides the same simplicity as LatchedSubscriber without confusing
// semantics of faking a pointer type.
// 
// This is an extension of the swri::Subscriber class to simplify the
// common case where you only care about the most recent value of a
// message rather than trying to queue up messages or responding to
// events.  It has its own callback that is provided to
// swri::Subscriber and simply stores the message for the user to
// access whenever necessary.
template <class M>
class LatchedSubscriber : public Subscriber
{
 private:
  template <class M2>
  struct LatchedReceiver
  {
    boost::shared_ptr<M2 const> msg_;    
    void handleMessage(const boost::shared_ptr<M2 const> &msg) {
      msg_ = msg;
    }    
  };  // struct LatchedReceiver

  boost::shared_ptr<LatchedReceiver<M> > receiver_;
  M empty_;
  
 public:
  LatchedSubscriber();

  // Create a new latched subscriber.  This is most like the
  // swri::Subscriber interface, but you have to provide the template
  // argument twice (once in your class declaration and at
  // construction).  See the initialize() method for a simpler
  // alternative.
  LatchedSubscriber(ros::NodeHandle &nh,
                    const std::string &topic,
                    const ros::TransportHints &transport_hints=ros::TransportHints());

  // Creates a latched subscriber in place.  This is more convenient
  // because you don't have to provide the template argument a second
  // time.
  void initialize(ros::NodeHandle &nh,
                  const std::string &topic,
                  const ros::TransportHints &transport_hints=ros::TransportHints());

  LatchedSubscriber<M>& operator=(const LatchedSubscriber<M> &other);

  // Return the value of the most recent message.  This is guaranteed
  // to be non-NULL if the messageCount() is non-zero, otherwise it
  // may be null.
  const boost::shared_ptr<M const>& data() const;
  M const * operator->() const;
  
  void reset();  
};  // class LatchedSubscriber

template<class M>
LatchedSubscriber<M>::LatchedSubscriber()
{
  // Setup an empty receiver so that we can assume receiver_ is
  // non-null and avoid a lot of unnecessary NULL checks.
  receiver_ = boost::shared_ptr<LatchedReceiver<M> >(new LatchedReceiver<M>());
}

template<class M>
LatchedSubscriber<M>::LatchedSubscriber(
  ros::NodeHandle &nh,
  const std::string &topic,
  const ros::TransportHints &transport_hints)
{
  ROS_WARN("swri_roscpp::LatchedSubscriber has been deprecated.  See header for information.");
  receiver_ = boost::shared_ptr<LatchedReceiver<M> >(new LatchedReceiver<M>());
  // It seems like there should be a better way to do this?
  Subscriber::operator=(
    Subscriber(nh, topic, 1,
               &LatchedReceiver<M>::handleMessage, receiver_.get(), transport_hints));
}

template<class M>
void LatchedSubscriber<M>::initialize(
  ros::NodeHandle &nh,
  const std::string &topic,
  const ros::TransportHints &transport_hints)
{
  *this = LatchedSubscriber<M>(nh, topic, transport_hints);
}


template<class M>
LatchedSubscriber<M>& LatchedSubscriber<M>::operator=(const LatchedSubscriber<M> &other)
{
  Subscriber::operator=(other);
  receiver_ = other.receiver_;
  return *this;
}
    
template<class M>
const boost::shared_ptr<M const>& LatchedSubscriber<M>::data() const
{
  return receiver_->msg_;
}

template<class M>
M const * LatchedSubscriber<M>::operator->() const
{
  if (receiver_->msg_) {
    return receiver_->msg_.get();
  } else {
    return &empty_;
  }
}

template<class M>
void LatchedSubscriber<M>::reset()
{
  receiver_->msg_.reset();
  resetStatistics();
}
}  // namespace swri
#endif  // SWRI_ROSCPP_SUBSCRIBER_H_

