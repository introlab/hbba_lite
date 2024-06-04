#ifndef HBBA_LITE_CORE_DESIRE_H
#define HBBA_LITE_CORE_DESIRE_H

#include <cstdint>
#include <atomic>
#include <memory>
#include <typeindex>
#include <type_traits>

class Desire;

class DesireType
{
    std::type_index m_type;

    explicit DesireType(std::type_index type);

public:
    template<class T>
    static DesireType get();
    static DesireType null();

    bool operator==(const DesireType& other) const;
    bool operator!=(const DesireType& other) const;
    bool operator<(const DesireType& other) const;
    bool operator<=(const DesireType& other) const;
    bool operator>(const DesireType& other) const;
    bool operator>=(const DesireType& other) const;

    const char* name() const;
    std::size_t hashCode() const;
};

template<class T>
inline DesireType DesireType::get()
{
    static_assert(std::is_base_of<Desire, T>::value, "T must be a subclass of Desire");
    return DesireType(std::type_index(typeid(T)));
}

inline DesireType DesireType::null()
{
    return DesireType(std::type_index(typeid(std::nullptr_t)));
}

inline bool DesireType::operator==(const DesireType& other) const
{
    return m_type == other.m_type;
}

inline bool DesireType::operator!=(const DesireType& other) const
{
    return m_type != other.m_type;
}

inline bool DesireType::operator<(const DesireType& other) const
{
    return m_type < other.m_type;
}

inline bool DesireType::operator<=(const DesireType& other) const
{
    return m_type <= other.m_type;
}

inline bool DesireType::operator>(const DesireType& other) const
{
    return m_type > other.m_type;
}

inline bool DesireType::operator>=(const DesireType& other) const
{
    return m_type >= other.m_type;
}

inline const char* DesireType::name() const
{
    return m_type.name();
}

inline std::size_t DesireType::hashCode() const
{
    return m_type.hash_code();
}

namespace std
{
    template<>
    struct hash<DesireType>
    {
        inline std::size_t operator()(const DesireType& type) const { return type.hashCode(); }
    };
}

#define DECLARE_DESIRE_METHODS(className)                                                                              \
    std::unique_ptr<Desire> clone() override                                                                           \
    {                                                                                                                  \
        return std::make_unique<className>(*this);                                                                     \
    }                                                                                                                  \
    DesireType type() override                                                                                         \
    {                                                                                                                  \
        return DesireType::get<className>();                                                                           \
    }

class Desire
{
    static std::atomic_uint64_t m_idCounter;

    uint64_t m_id;
    uint16_t m_intensity;
    bool m_enabled;

public:
    explicit Desire(uint16_t intensity);
    virtual ~Desire();

    uint64_t id() const;
    uint16_t intensity() const;
    bool enabled() const;

    virtual std::unique_ptr<Desire> clone() = 0;
    virtual DesireType type() = 0;

    void enable();
    void disable();
};

inline uint64_t Desire::id() const
{
    return m_id;
}

inline uint16_t Desire::intensity() const
{
    return m_intensity;
}

inline bool Desire::enabled() const
{
    return m_enabled;
}

inline void Desire::enable()
{
    m_enabled = true;
}

inline void Desire::disable()
{
    m_enabled = false;
}

#endif
