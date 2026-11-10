#include "brain/tasks/communicationTask.hpp"

namespace brain {

CommunicationTask::CommunicationTask(SerialLink& link) : link_(link) {}

bool CommunicationTask::start() { return link_.start(); }

void CommunicationTask::stop() { link_.stop(); }

} // namespace brain
