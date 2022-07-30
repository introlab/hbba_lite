#ifndef HBBA_LITE_CORE_DESIRE_SET_H
#define HBBA_LITE_CORE_DESIRE_SET_H

#include <hbba_lite/utils/ClassMacros.h>
#include <hbba_lite/core/Desire.h>

#include <memory>
#include <unordered_map>
#include <mutex>
#include <vector>
#include <unordered_set>

class DesireSet;

class DesireSetTransaction
{
    DesireSet& m_desireSet;
    std::unique_lock<std::recursive_mutex> m_lock;

private:
    DesireSetTransaction(DesireSet& desireSet, std::unique_lock<std::recursive_mutex>&& lock);
    DECLARE_NOT_COPYABLE(DesireSetTransaction);

public:
    DesireSetTransaction(DesireSetTransaction&&) = default;
    ~DesireSetTransaction();

    friend DesireSet;
};

class DesireSetObserver
{
public:
    virtual void onDesireSetChanged(const std::vector<std::unique_ptr<Desire>>& enabledDesires) = 0;
};

class DesireSet
{
    std::unordered_map<uint64_t, std::unique_ptr<Desire>> m_desiresById;
    std::unordered_set<DesireSetObserver*> m_observers;

    std::recursive_mutex m_observerMutex;
    std::recursive_mutex m_desireMutex;
    bool m_isTransactionStarted;
    bool m_hasChanged;

    uint64_t m_updateCount;  // To monitor changes during observer calls

public:
    DesireSet();
    DECLARE_NOT_COPYABLE(DesireSet);
    DECLARE_NOT_MOVABLE(DesireSet);

    void addObserver(DesireSetObserver* observer);
    void removeObserver(DesireSetObserver* observer);

    DesireSetTransaction beginTransaction();

    template<class D, class... Types>
    uint64_t addDesire(Types... args);
    uint64_t addDesire(std::unique_ptr<Desire>&& desire);

    void removeDesire(uint64_t id);
    void clear();

    template <class T>
    void removeAllDesiresOfType();
    void removeAllDesiresOfType(std::type_index type);

    template <class T>
    bool containsAnyDesiresOfType();
    bool containsAnyDesiresOfType(std::type_index type);
    bool contains(uint64_t id);

    void enableAllDesires();
    void disableAllDesires();

private:
    void endTransaction(std::unique_lock<std::recursive_mutex> lock);
    void callObservers(std::unique_lock<std::recursive_mutex> desireLock);

    std::vector<std::unique_ptr<Desire>> getEnabledDesires();
    friend DesireSetTransaction;
};

template<class D, class... Types>
uint64_t DesireSet::addDesire(Types... args)
{
    return addDesire(std::make_unique<D>(&args...));
}

template <class T>
void DesireSet::removeAllDesiresOfType()
{
    removeAllDesiresOfType(std::type_index(typeid(T)));
}

template <class T>
bool DesireSet::containsAnyDesiresOfType()
{
    return containsAnyDesiresOfType(std::type_index(typeid(T)));
}

#endif
