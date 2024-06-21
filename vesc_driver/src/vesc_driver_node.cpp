#include <vesc_driver/vesc_driver.hpp>

int main(int argc, char** argv)
{
   if (argc != 2)
   {
      std::cerr << "Usage: " << argv[0] << " <vesc_config_file_absolute_path>\n";
      return 1;
   }

   const std::string configFilePath = argv[1];

   vesc_driver::VescDriver vd(configFilePath);

   while (true)
   {
      std::this_thread::sleep_for(std::chrono::seconds(2));
   }
}
