#pragma once

#include <corium/Runtime.hpp>
#include "corium_sim/app/SimAgentApp.hpp"
#include "corium_sim/events/SimEvents.hpp"

namespace corium_sim::runtime {

/// @brief Corium Agent Runtime managing MPSC event loops, services, and agent applications.
using AgentRuntime = corium::BasicRuntime<
    DefaultSimEvents,
    corium::BoundedMpscQueuePolicy<DefaultSimEvents, 1024>,
    corium::NoSignalPolicy,
    corium::DefaultStoragePolicy,
    corium::DropNewestOverflowPolicy,
    corium::DefaultTimerStoragePolicy
>;

} // namespace corium_sim::runtime

namespace corium_sim {
    using AgentRuntime = runtime::AgentRuntime;
}
