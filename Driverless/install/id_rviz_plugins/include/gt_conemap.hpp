// #ifndef OBJECT_DETECTION__BOUNDING_BOX_ARRAY_DISPLAY_HPP_
// #define OBJECT_DETECTION__BOUNDING_BOX_ARRAY_DISPLAY_HPP_

#include <rviz_common/display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_default_plugins/displays/marker/marker_common.hpp>
#include <rviz_default_plugins/displays/marker_array/marker_array_display.hpp>
#include <imperial_driverless_interfaces/msg/cone_map.hpp>
#include <memory>

// using autoware::common::types::float32_t;

namespace id
{
    namespace rviz_plugins
    {

        class ConeMapDisplay
            : public rviz_common::RosTopicDisplay<imperial_driverless_interfaces::msg::ConeMap>
        {
            Q_OBJECT

        public:
            using MarkerCommon = rviz_default_plugins::displays::MarkerCommon;
            using Marker = visualization_msgs::msg::Marker;
            using ConeMap = imperial_driverless_interfaces::msg::ConeMap;
            using Cone = imperial_driverless_interfaces::msg::Cone;

            ConeMapDisplay();
            void onInitialize() override;
            void load(const rviz_common::Config &config) override;
            void update(float wall_dt, float ros_dt) override;
            void reset() override;

        private Q_SLOTS:
            void updateProperty();

        private:
            std_msgs::msg::ColorRGBA BLUE;
            std_msgs::msg::ColorRGBA YELLOW;
            std_msgs::msg::ColorRGBA ORANGE;

            double BLUE_SCALE = 0.2;
            double YELLOW_SCALE = 0.2;
            double BIG_ORANGE_SCALE = 0.25;
            double SMALL_ORANGE_SCALE = 0.15;
            // Convert boxes into markers, push them to the display queue
            void processMessage(imperial_driverless_interfaces::msg::ConeMap::ConstSharedPtr array) override;
            // Convert box message to a marker message
            visualization_msgs::msg::Marker::SharedPtr get_marker(const imperial_driverless_interfaces::msg::Cone &cone, unsigned int id, std_msgs::msg::ColorRGBA colour, double scale, std_msgs::msg::Header header) const;

            std::unique_ptr<MarkerCommon> m_marker_common;
            imperial_driverless_interfaces::msg::ConeMap::ConstSharedPtr msg_cache{};
            // rviz_common::properties::ColorProperty *no_label_color_property_;
            // rviz_common::properties::ColorProperty *car_color_property_;
            // rviz_common::properties::ColorProperty *pedestrian_color_property_;
            // rviz_common::properties::ColorProperty *cyclist_color_property_;
            // rviz_common::properties::ColorProperty *motorcycle_color_property_;
            // rviz_common::properties::ColorProperty *other_color_property_;
            // rviz_common::properties::FloatProperty *alpha_property_;
        };
    } // namespace rviz_plugins
} // namespace id

// #endif // OBJECT_DETECTION__BOUNDING_BOX_ARRAY_DISPLAY_HPP_
