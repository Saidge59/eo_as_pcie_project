// shared_state.h
#ifndef SHARED_STATE_H
#define SHARED_STATE_H

#include <atomic>
#include <semaphore.h>

/**
 * @class shared_state
 * @brief Manages shared state information for the application
 */
class shared_state
{
public:
    std::atomic<bool> service_running{false}; ///< Flag indicating if service is running
    sem_t stop_event;                         ///< Semaphore for stop event

    /**
     * @brief Constructor
     */
    shared_state()
    {
        sem_init(&stop_event, 0, 0); // Initialize semaphore, not shared between processes, initial value 0
    }

    /**
     * @brief Destructor
     */
    ~shared_state()
    {
        sem_destroy(&stop_event); // Clean up semaphore
    }
};

#endif // SHARED_STATE_H