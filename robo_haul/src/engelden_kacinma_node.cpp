#include <algorithm>
#include <cmath>    
#include <limits>    
#include <memory>    
#include <string>    

#include "rclcpp/rclcpp.hpp"              // مكتبة ROS2 الأساسية للـ Node
#include "sensor_msgs/msg/laser_scan.hpp" 
#include "geometry_msgs/msg/twist.hpp"    


class EngeldenKacinmaNode : public rclcpp::Node
{
public:
    // Constructor: يتم تنفيذه عند تشغيل الـ node
    EngeldenKacinmaNode() : Node("engelden_kacinma_node")
    {
        // تعريف البارامترات مع قيم افتراضية
        this->declare_parameter("scan_topic", "/scan");       // اسم topic الليدار
        this->declare_parameter("cmd_vel_topic", "/cmd_vel"); // اسم topic أوامر الحركة
        this->declare_parameter("obstacle_distance", 0.7);    // أقل مسافة آمنة أمام الروبوت
        this->declare_parameter("side_safe_distance", 0.55);  // أقل مسافة آمنة على الجانبين
        this->declare_parameter("linear_speed", 0.25);        // السرعة الخطية للأمام
        this->declare_parameter("angular_speed", 0.5);        // السرعة الزاوية عند الدوران

        // قراءة قيم البارامترات وتخزينها داخل متغيرات الكلاس
        scan_topic_ = this->get_parameter("scan_topic").as_string();
        cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
        obstacle_distance_ = this->get_parameter("obstacle_distance").as_double();
        side_safe_distance_ = this->get_parameter("side_safe_distance").as_double();
        linear_speed_ = this->get_parameter("linear_speed").as_double();
        angular_speed_ = this->get_parameter("angular_speed").as_double();

        // إنشاء Publisher لإرسال أوامر الحركة للروبوت
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

        // إنشاء Subscriber للاشتراك ببيانات الليدار
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, // الموضوع الذي سنقرأ منه
            10,          // حجم الـ queue
            std::bind(&EngeldenKacinmaNode::scanCallback, this, std::placeholders::_1));
        // عند وصول Scan جديد يتم استدعاء scanCallback

        // رسالة تظهر عند بدء تشغيل الـ node
        RCLCPP_INFO(this->get_logger(), "Obstacle avoidance node started.");
    }

private:
    // دالة لحساب أقل مسافة ضمن مجال زوايا معين من الليدار
    double getMinRange(const sensor_msgs::msg::LaserScan &scan, double min_angle, double max_angle)
    {
        // نبدأ بقيمة لا نهائية حتى نستطيع المقارنة واختيار الأصغر
        double min_dist = std::numeric_limits<double>::infinity();

        // المرور على جميع قيم الليدار
        for (size_t i = 0; i < scan.ranges.size(); i++)
        {
            // حساب الزاوية المقابلة لكل قراءة
            double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;

            // المسافة المقروءة من الليدار
            double r = scan.ranges[i];

            // إذا كانت الزاوية ضمن المجال المطلوب
            if (angle >= min_angle && angle <= max_angle)
            {
                // إذا كانت القراءة صحيحة وضمن حدود الليدار
                if (std::isfinite(r) && r >= scan.range_min && r <= scan.range_max)
                {
                    // اختيار أصغر مسافة ضمن هذا المجال
                    min_dist = std::min(min_dist, r);
                }
            }
        }

        // إرجاع أقل مسافة موجودة
        return min_dist;
    }

    // هذه الدالة تُستدعى كلما وصلت رسالة LaserScan جديدة
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // إذا كانت القراءات فارغة نوقف الروبوت مباشرة
        if (msg->ranges.empty())
        {
            stopRobot();
            return;
        }

        // تقسيم مجال الليدار إلى 3 مناطق:
        // front: المنطقة الأمامية
        // left : المنطقة اليسرى
        // right: المنطقة اليمنى
        double front = getMinRange(*msg, -0.35, 0.35);  // أمام الروبوت
        double left = getMinRange(*msg, 0.35, 1.57);    // يسار الروبوت
        double right = getMinRange(*msg, -1.57, -0.35); // يمين الروبوت

        // متغير أوامر الحركة الذي سنرسله للروبوت
        geometry_msgs::msg::Twist cmd;

        // طباعة القيم كل ثانية بدل الطباعة المستمرة السريعة
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "front=%.2f left=%.2f right=%.2f",
            front, left, right);

        // طباعة تحذيرات حسب مكان وجود العائق
        if (front < obstacle_distance_)
        {
            RCLCPP_WARN(this->get_logger(), "ÖNDE ENGEL VAR!");
        }
        else if (left < side_safe_distance_)
        {
            RCLCPP_WARN(this->get_logger(), "SOL TARAFTA ENGEL VAR!");
        }
        else if (right < side_safe_distance_)
        {
            RCLCPP_WARN(this->get_logger(), "SAĞ TARAFTA ENGEL VAR!");
        }

        // منطق اتخاذ القرار للحركة

        // 1) إذا كان هناك عائق أمام الروبوت
        if (std::isfinite(front) && front < obstacle_distance_)
        {
            cmd.linear.x = 0.03; // يتحرك ببطء شديد للأمام أثناء الدوران حتى لا يعلق

            // إذا كان اليسار أوسع من اليمين، لف يسار
            if (left > right)
            {
                cmd.angular.z = angular_speed_;
            }
            // وإلا لف يمين
            else
            {
                cmd.angular.z = -angular_speed_;
            }
        }
        // 2) إذا كان هناك عائق قريب من الجهة اليسرى
        else if (std::isfinite(left) && left < side_safe_distance_)
        {
            cmd.linear.x = linear_speed_ * 0.6; // تقليل السرعة
            cmd.angular.z = -0.35;              // الانعطاف يمين للابتعاد عن العائق الأيسر
        }
        // 3) إذا كان هناك عائق قريب من الجهة اليمنى
        else if (std::isfinite(right) && right < side_safe_distance_)
        {
            cmd.linear.x = linear_speed_ * 0.6; // تقليل السرعة
            cmd.angular.z = 0.35;               // الانعطاف يسار للابتعاد عن العائق الأيمن
        }
        // 4) إذا كان الطريق مفتوحًا
        else
        {
            cmd.linear.x = linear_speed_; // تقدم للأمام
            cmd.angular.z = 0.0;          // بدون دوران
        }

        // نشر أوامر الحركة على topic /cmd_vel
        cmd_pub_->publish(cmd);
    }

    // دالة لإيقاف الروبوت بالكامل
    void stopRobot()
    {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;  // إيقاف الحركة الأمامية
        cmd.angular.z = 0.0; // إيقاف الدوران
        cmd_pub_->publish(cmd);
    }

    // Subscriber لاستقبال بيانات الليدار
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    // Publisher لإرسال أوامر الحركة
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    // أسماء الـ topics
    std::string scan_topic_;
    std::string cmd_vel_topic_;

    // متغيرات التحكم
    double obstacle_distance_;  // مسافة الأمان الأمامية
    double side_safe_distance_; // مسافة الأمان الجانبية
    double linear_speed_;       // السرعة الخطية
    double angular_speed_;      // السرعة الزاوية
};

// الدالة الرئيسية main
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // تهيئة ROS2
    rclcpp::spin(std::make_shared<EngeldenKacinmaNode>());
    // إنشاء الـ node وتشغيلها بشكل مستمر
    rclcpp::shutdown(); // إغلاق ROS2 عند الانتهاء
    return 0;
}