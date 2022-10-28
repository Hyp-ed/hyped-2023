#include <array>

namespace hyped::navigation{

  class Crosscheck{

    public:

      Crosscheck();

      bool checkAll();

    private:

      bool checkEncoderImu();

      bool checkEncoderCamera();

      bool checkEncooderKeyence();

  };
}