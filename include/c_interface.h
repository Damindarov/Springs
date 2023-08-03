#ifndef C_INTERFACE_H
#define C_INTERFACE_H

#include <stdint.h>

#ifdef  __cplusplus
extern "C" {
#endif
#define WINDOWS_DLL_API   __declspec( dllimport )

    struct WINDOWS_DLL_API Point {
        float x;
        float y;
        float z;
    };

    struct WINDOWS_DLL_API PointCloud {
        unsigned int size;
        Point* data;
    };

    struct WINDOWS_DLL_API DepthImage {
        unsigned int height;
        unsigned int width;
        
        float z_unit_mm;

        float x_min;
        float x_max;
        float y_min;
        float y_max;

        uint16_t* data;
    };

    struct WINDOWS_DLL_API BoolResult
    {
        bool valid;
        bool value;
    };

    struct WINDOWS_DLL_API Uint32Result
    {
        bool valid;
        uint32_t value;
    };

bool	WINDOWS_DLL_API lib_init();
bool	WINDOWS_DLL_API lib_initialized();
bool	WINDOWS_DLL_API lib_clear();

//bool	WINDOWS_DLL_API restart_ethercat_data_exchange();

bool    WINDOWS_DLL_API set_scanner_param(const char* name, uint32_t value);
Uint32Result  WINDOWS_DLL_API get_scanner_param(const char* name);

bool    WINDOWS_DLL_API set_discrete_output(int ind, bool output);
BoolResult  WINDOWS_DLL_API get_discrete_input(int ind);

/*
# get_driver_status_word
# return : the driver status word if possible
libc.get_driver_status_word.restype = UINT32_OPTIONAL
libc.get_driver_status_word.argtypes = []
get_driver_status_word = libc.get_driver_status_word

# set_driver_ctrl_word
# arg0 : value of ctrl word
# return : true if success, false otherwise
libc.set_driver_ctrl_word.restype = ctypes.c_bool
libc.set_driver_ctrl_word.argtypes = [ctypes.c_uint32]
set_driver_ctrl_word = libc.set_driver_ctrl_word
*/

Uint32Result WINDOWS_DLL_API get_driver_status_word();
bool WINDOWS_DLL_API set_driver_ctrl_word(uint32_t ctrl_word);


bool WINDOWS_DLL_API driver_fault();
void WINDOWS_DLL_API clear_driver_fault();

bool WINDOWS_DLL_API driver_hold_off();

bool	WINDOWS_DLL_API do_homing();

// возвращает текущую позицию или -1, если позицию определить невозможно
double	WINDOWS_DLL_API position_mm();

double	WINDOWS_DLL_API max_position_mm();

// возвращает true, если задание выполнено и false, если оно не выполнено или его выполнить невозможно
// velocity_koeff [0, 1]
bool	WINDOWS_DLL_API move_to(double position_mm, double velocity_koeff);

// возвращает текущую позицию или -1, если позицию определить невозможно
bool	WINDOWS_DLL_API scan(double marchSpeed_koef, double scanSpeed_koef, double beginPosition_mm, double endPosition_mm, double parkingPosition_mm);

bool	WINDOWS_DLL_API save_scan(const wchar_t* path_to_file);
bool	WINDOWS_DLL_API load_scan(const wchar_t* path_to_file);

PointCloud WINDOWS_DLL_API get_point_cloud();
void WINDOWS_DLL_API delete_point_cloud(PointCloud cloud);

DepthImage WINDOWS_DLL_API get_depth_image(unsigned int width, unsigned int height);
void WINDOWS_DLL_API delete_depth_image(DepthImage* image);

// функция калибровки
bool	WINDOWS_DLL_API calibrate(unsigned int plate0_ind, unsigned int plate0_orientation,
                                  unsigned int plate1_ind, unsigned int plate1_orientation,
                                  unsigned int plate2_ind, unsigned int plate2_orientation,
                                  unsigned int plate3_ind, unsigned int plate4_orientation,
                                  double x01, double y01, double z01,
                                  double x02, double y02, double z02,
                                  double x03, double y03, double z03,
                                  double x04, double y04, double z04,
                                  double x11, double y11, double z11,
                                  double x12, double y12, double z12,
                                  double x13, double y13, double z13,
                                  double x14, double y14, double z14,
                                  double x21, double y21, double z21,
                                  double x22, double y22, double z22,
                                  double x23, double y23, double z23,
                                  double x24, double y24, double z24,
                                  double x31, double y31, double z31,
                                  double x32, double y32, double z32,
                                  double x33, double y33, double z33,
                                  double x34, double y34, double z34,
                                  double*  m00, double*  m01, double*  m02, double*  m03,
                                  double*  m10, double*  m11, double*  m12, double*  m13,
                                  double*  m20, double*  m21, double*  m22, double*  m23,
                                  double*  m30, double*  m31, double*  m32, double*  m33);


#ifdef  __cplusplus
}
#endif

#endif // C_INTERFACE_H
