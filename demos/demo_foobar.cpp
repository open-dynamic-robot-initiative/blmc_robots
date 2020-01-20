#include <robot_interfaces/finger_types.hpp>
#include <blmc_robots/fake_finger_driver.hpp>

using namespace robot_interfaces;
using namespace blmc_robots;

int main()
{
    try {
    auto finger_data = std::make_shared<FingerTypes::MultiProcessData>("foo", 1000);
    //auto finger_data = std::make_shared<FingerTypes::SingleProcessData>(1000);
    auto fake_finger_backend = create_fake_finger_backend(finger_data);

    auto finger = FingerTypes::Frontend(finger_data);

    fake_finger_backend->initialize();

    std::cout << "Start... " << std::endl;
    FingerTypes::Vector desired_torque = FingerTypes::Vector::Zero();
    while (true) {
        FingerTypes::Vector current_position;
        for (int i = 0; i < 300; i++) {
            auto action = FingerTypes::Action::Torque(desired_torque);
            auto t = finger.append_desired_action(action);

            current_position = finger.get_observation(t).position;
        }
        std::cout << "Position: " << current_position.transpose() << std::endl;
    }

    } catch (std::runtime_error e) {
        std::cerr << "Exception MAIN " << e.what() << std::endl;
    }

    return 0;
}
