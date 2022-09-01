#include <hbba_lite/core/Desire.h>

using namespace std;

DesireType::DesireType(type_index type) : m_type(type) {}

atomic_uint64_t Desire::m_idCounter(0);

Desire::Desire(uint16_t intensity) : m_id(m_idCounter.fetch_add(1)), m_intensity(intensity), m_enabled(true) {}

Desire::~Desire() {}
