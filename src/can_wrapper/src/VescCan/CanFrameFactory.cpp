#include "can_wrapper/VescCan/CanFrameFactory.hpp"

namespace _vccp = VescCan::ConstsPacked;

template <class T>
const T VescCan::CanFrameFactory<T>::sConsts;

template class VescCan::CanFrameFactory<_vccp::SetDuty>;
template class VescCan::CanFrameFactory<_vccp::SetCurrent>;
template class VescCan::CanFrameFactory<_vccp::SetCurrentBrake>;
template class VescCan::CanFrameFactory<_vccp::SetRpm>;
template class VescCan::CanFrameFactory<_vccp::SetPos>;
template class VescCan::CanFrameFactory<_vccp::SetCurrentRel>;
template class VescCan::CanFrameFactory<_vccp::SetCurrentBreakeRel>;
template class VescCan::CanFrameFactory<_vccp::SendCurrentHandbrake>;
template class VescCan::CanFrameFactory<_vccp::SendCurrentHandbrakeRel>;

