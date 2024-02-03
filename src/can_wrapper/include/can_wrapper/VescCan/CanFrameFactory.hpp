#ifndef Vesc_CanFrameFactory_h_
#define Vesc_CanFrameFactory_h_

#include "CanFrame.hpp"
#include <type_traits>

namespace VescCan::ConstsPacked
{

    template< typename NT >
    class SendCommand
    {
    public:
        typedef NT nativeArg_t;
        virtual VescCan::Consts::Command getCommandId() const noexcept = 0;
        virtual int32_t getCommandArgScale() const noexcept = 0;
    };
    
    class SetDuty : public SendCommand<float>
    {
    public:
        virtual VescCan::Consts::Command getCommandId() const noexcept override { return VescCan::Consts::Command::SET_DUTY; }
        virtual int32_t getCommandArgScale() const noexcept override { return VescCan::Consts::SET_DUTY_SCALE; };
    };

    class SetCurrent : public SendCommand<int32_t>
    {
    public:
        virtual VescCan::Consts::Command getCommandId() const noexcept override { return VescCan::Consts::Command::SET_CURRENT; }
        virtual int32_t getCommandArgScale() const noexcept override { return VescCan::Consts::SET_CURRENT_SCALE; };
    };

    class SetCurrentBrake : public SendCommand<int32_t>
    {
    public:
        virtual VescCan::Consts::Command getCommandId() const noexcept override { return VescCan::Consts::Command::SET_CURRENT_BRAKE; }
        virtual int32_t getCommandArgScale() const noexcept override { return VescCan::Consts::SET_CURRENT_BRAKE_SCALE; };
    };
    
    class SetRpm : public SendCommand<int32_t>
    {
    public:
        virtual VescCan::Consts::Command getCommandId() const noexcept override { return VescCan::Consts::Command::SET_RPM; }
        virtual int32_t getCommandArgScale() const noexcept override { return VescCan::Consts::SET_RPM_SCALE; };
    };

    class SetPos : public SendCommand<int32_t>
    {
    public:
        virtual VescCan::Consts::Command getCommandId() const noexcept override { return VescCan::Consts::Command::SET_POS; }
        virtual int32_t getCommandArgScale() const noexcept override { return VescCan::Consts::SET_POS_SCALE; };
    };

    class SetCurrentRel : public SendCommand<float>
    {
    public:
        virtual VescCan::Consts::Command getCommandId() const noexcept override { return VescCan::Consts::Command::SET_CURRENT_REL; }
        virtual int32_t getCommandArgScale() const noexcept override { return VescCan::Consts::SET_CURRENT_REL_SCALE; };
    };

    class SetCurrentBreakeRel : public SendCommand<float>
    {
    public:
        virtual VescCan::Consts::Command getCommandId() const noexcept override { return VescCan::Consts::Command::SET_CURRENT_BRAKE_REL; }
        virtual int32_t getCommandArgScale() const noexcept override { return VescCan::Consts::SET_CURRENT_BRAKE_REL_SCALE; };
    };

    class SendCurrentHandbrake : public SendCommand<int32_t>
    {
    public:
        virtual VescCan::Consts::Command getCommandId() const noexcept override { return VescCan::Consts::Command::SET_CURRENT_HANDBRAKE; }
        virtual int32_t getCommandArgScale() const noexcept override { return VescCan::Consts::SET_CURRENT_HANDBRAKE_SCALE; };
    };

    class SendCurrentHandbrakeRel : public SendCommand<float>
    {
    public:
        virtual VescCan::Consts::Command getCommandId() const noexcept override { return VescCan::Consts::Command::SET_CURRENT_HANDBRAKE_REL; }
        virtual int32_t getCommandArgScale() const noexcept override { return VescCan::Consts::SET_CURRENT_HANDBRAKE_REL_SCALE; };
    };
}

namespace VescCan
{
    template<class T>
    class CanFrameFactory
    {
    private:
        typedef typename T::nativeArg_t nativeArg_t;

        static_assert(std::is_base_of<VescCan::ConstsPacked::SendCommand<nativeArg_t>,T>().value,"T must derive from SendCommand");

        static const T sConsts;
    public:

        CanFrameFactory() = delete;               
        CanFrameFactory(CanFrameFactory&) = delete;        
        CanFrameFactory(CanFrameFactory&&) = delete;        
        ~CanFrameFactory() = delete;  

        /// @brief Creates ready to send CanFrame
        /// @param vescID vesc id
        /// @param commandArg native type argument, that will be multiplied by scale, and converted to Big Endian
        static CanFrame create(uint8_t vescID, nativeArg_t commandArg)
        {
            int32_t scaled = static_cast<int32_t>(commandArg * sConsts.getCommandArgScale());
            return CanFrame(vescID, sConsts.getCommandId(), boost::endian::big_int32_buf_t(scaled));
        }

        static nativeArg_t getUnscaledNative(CanFrame cf)
        {
            nativeArg_t deendianed = static_cast<nativeArg_t>(cf.commandArg.value());
            return deendianed / sConsts.getCommandArgScale();
        }
           
    };
}

#endif // Vesc_FrameCommand_h_