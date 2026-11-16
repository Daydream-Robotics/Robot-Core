#include "brain/tasks/communicationTask.hpp"

namespace brain {

CommunicationTask::CommunicationTask(SerialLink& link) : m_link(link) {}

bool CommunicationTask::start() { return m_link.start(); }

void CommunicationTask::stop() { m_link.stop(); }

} // namespace brain
