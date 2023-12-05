#ifndef NOTIFICATION_SERVICE_H__
#define NOTIFICATION_SERVICE_H__

#include <condition_variable>
#include <map>
#include <mutex>
#include <thread>
#include <unordered_map>

#define MAX_NOTIFICATIONS_BUFFER 10240

typedef void NotificationHandler(const std::map<int, int>&);

typedef boost::signals2::signal<NotificationHandler> SIGNAL;
typedef std::pair<SIGNAL*, const std::map<std::string, std::string>>
    SignalAndNotificationPair;
typedef std::vector<SignalAndNotificationPair> QueuedNotificationVector;

class NotificationService : public INotificationService {
 public:
  static INotificationService* GetNotificationService();
  ~NotificationService();

  void Publish(const std::string& channel,
               const std::map<std::string, std::string>&
                   std::map<std::string, std::string>);
  SubscriptionToken Subscribe(
      const std::string& channel,
      const std::function<NotificationHandler>& fnNotificationHandler);
  void Unsubscribe(SubscriptionToken& subscriptionToken);

 private:
  NotificationService();
  void NotificationLoop();
  void FlushQueuedNotifications();
  void QueueNotification(SIGNAL* pSignal,
                         const std::map<std::string, std::string>&
                             std::map<std::string, std::string>);

 private:
  std::mutex m_mutexQueuedNotificationVector;
  std::mutex m_mutexNotificationLoop;
  std::condition_variable m_cvPauseFlushThread;
  bool m_bShutdown;
  std::thread m_FlushNotificationsThread;

  // Queue of Notifications
  QueuedNotificationVector m_vecQueuedNotifications;
  std::map<std::string, SIGNAL*> m_mapSignal;

  static NotificationService* s_pNotificationService;
};

#endif /*__NOTIFICATION_SERVICE_H__*/