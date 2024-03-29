#include <hbba_lite/core/DesireSet.h>

#include <algorithm>

using namespace std;

DesireSetTransaction::DesireSetTransaction(DesireSet& desireSet, unique_lock<recursive_mutex>&& lock)
    : m_desireSet(desireSet),
      m_lock(move(lock))
{
}

DesireSetTransaction::~DesireSetTransaction()
{
    if (m_lock.owns_lock())
    {
        m_desireSet.endTransaction(move(m_lock));
    }
}

DesireSet::DesireSet() : m_isTransactionStarted(false), m_hasChanged(false), m_updateCount(0) {}

void DesireSet::addObserver(DesireSetObserver* observer)
{
    lock_guard<recursive_mutex> lock(m_observerMutex);
    m_observers.emplace(observer);
}

void DesireSet::removeObserver(DesireSetObserver* observer)
{
    lock_guard<recursive_mutex> lock(m_observerMutex);
    m_observers.erase(observer);
}

DesireSetTransaction DesireSet::beginTransaction()
{
    unique_lock<recursive_mutex> lock(m_desireMutex);
    m_isTransactionStarted = true;
    return move(DesireSetTransaction(*this, move(lock)));
}

uint64_t DesireSet::addDesire(unique_ptr<Desire>&& desire)
{
    unique_lock<recursive_mutex> lock(m_desireMutex);

    uint64_t id = desire->id();
    m_desiresById[id] = move(desire);

    m_hasChanged = true;
    callObservers(move(lock));

    return id;
}

void DesireSet::removeDesire(uint64_t id)
{
    unique_lock<recursive_mutex> lock(m_desireMutex);
    if (m_desiresById.erase(id) == 1)
    {
        m_hasChanged = true;
    }
    callObservers(move(lock));
}

void DesireSet::clear()
{
    unique_lock<recursive_mutex> lock(m_desireMutex);
    if (!m_desiresById.empty())
    {
        m_hasChanged = true;
    }
    m_desiresById.clear();
    callObservers(move(lock));
}

void DesireSet::removeAllDesiresOfType(DesireType type)
{
    unique_lock<recursive_mutex> lock(m_desireMutex);
    size_t sizeBefore = m_desiresById.size();

    for (auto it = m_desiresById.begin(); it != m_desiresById.end();)
    {
        if (it->second->type() == type)
        {
            it = m_desiresById.erase(it);
        }
        else
        {
            ++it;
        }
    }

    if (sizeBefore != m_desiresById.size())
    {
        m_hasChanged = true;
    }
    callObservers(move(lock));
}

bool DesireSet::containsAnyDesiresOfType(DesireType type)
{
    unique_lock<recursive_mutex> lock(m_desireMutex);
    return any_of(m_desiresById.begin(), m_desiresById.end(), [=](const auto& p) { return p.second->type() == type; });
}

bool DesireSet::contains(uint64_t id)
{
    unique_lock<recursive_mutex> lock(m_desireMutex);
    auto it = m_desiresById.find(id);
    return it != m_desiresById.end();
}

void DesireSet::enableAllDesires()
{
    unique_lock<recursive_mutex> lock(m_desireMutex);

    for (auto& pair : m_desiresById)
    {
        if (!pair.second->enabled())
        {
            pair.second->enable();
            m_hasChanged = true;
        }
    }
    callObservers(move(lock));
}

void DesireSet::disableAllDesires()
{
    unique_lock<recursive_mutex> lock(m_desireMutex);

    for (auto& pair : m_desiresById)
    {
        if (pair.second->enabled())
        {
            pair.second->disable();
            m_hasChanged = true;
        }
    }
    callObservers(move(lock));
}

void DesireSet::endTransaction(unique_lock<recursive_mutex> lock)
{
    m_isTransactionStarted = false;
    callObservers(move(lock));
}

vector<unique_ptr<Desire>> DesireSet::getEnabledDesires()
{
    unique_lock<recursive_mutex> lock(m_desireMutex);
    if (m_isTransactionStarted)
    {
        return {};
    }

    vector<unique_ptr<Desire>> enabledDesires;
    for (auto& pair : m_desiresById)
    {
        if (pair.second->enabled())
        {
            enabledDesires.emplace_back(pair.second->clone());
        }
    }
    return enabledDesires;
}

void DesireSet::callObservers(unique_lock<recursive_mutex> desireLock)
{
    if (!m_hasChanged || m_isTransactionStarted)
    {
        return;
    }
    m_updateCount++;
    m_hasChanged = false;

    vector<unique_ptr<Desire>> enabledDesires = getEnabledDesires();

    uint64_t updateCountAtBeginning = m_updateCount;
    lock_guard<recursive_mutex> lock(m_observerMutex);
    for (auto& observer : m_observers)
    {
        if (updateCountAtBeginning != m_updateCount)
        {
            break;  // The previous observer has changed the desire set.
        }

        observer->onDesireSetChanged(enabledDesires);
    }
}
