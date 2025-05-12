/*==============================================================================
 * pid_controller.cpp -- A QSpice component implementation of a discrete PID
 * controller.
 *
 * Developed by QSpice forum members @KSKelvin and @RDunn.
 *
 * Revision History:
 * 2023.10.10 -- Initial release.
 * 2025.01.03 -- Modified by JY with integral anti-windup via Clamping
 *
 * To build with Digital Mars C++ Compiler:
 *    dmc -mn -WD pid_controller.cpp kernel32.lib
 *============================================================================*/

// Automatically generated C++ file on Fri Jan  3 14:20:55 2025
//
// To build with Digital Mars C++ Compiler:
//
//    dmc -mn -WD pid_controller.cpp kernel32.lib

#include <malloc.h>

extern "C" __declspec(dllexport) int (*Display)(const char *format, ...) = 0; // works like printf()
extern "C" __declspec(dllexport) const double *DegreesC                  = 0; // pointer to current circuit temperature
extern "C" __declspec(dllexport) const int *StepNumber                   = 0; // pointer to current step number
extern "C" __declspec(dllexport) const int *NumberSteps                  = 0; // pointer to estimated number of steps
extern "C" __declspec(dllexport) const char* const *InstanceName         = 0; // pointer to address of instance name
extern "C" __declspec(dllexport) const char *QUX                         = 0; // path to QUX.exe
extern "C" __declspec(dllexport) const bool *ForKeeps                    = 0; // pointer to whether being evaluated non-hypothetically
extern "C" __declspec(dllexport) const bool *HoldICs                     = 0; // pointer to whether instance initial conditions are being held
extern "C" __declspec(dllexport) int (*DFFT)(struct sComplex *u, bool inv, unsigned int N, double scale) = 0; // discrete Fast Fourier function

union uData
{
   bool b;
   char c;
   unsigned char uc;
   short s;
   unsigned short us;
   int i;
   unsigned int ui;
   float f;
   double d;
   long long int i64;
   unsigned long long int ui64;
   char *str;
   unsigned char *bytes;
};

// int DllMain() must exist and return 1 for a process to load the .DLL
// See https://docs.microsoft.com/en-us/windows/win32/dlls/dllmain for more information.
int __stdcall DllMain(void *module, unsigned int reason, void *reserved) { return 1; }

void bzero(void *ptr, unsigned int count)
{
   unsigned char *first = (unsigned char *) ptr;
   unsigned char *last  = first + count;
   while(first < last)
      *first++ = '\0';
}

// #undef pin names lest they collide with names in any header file(s) you might include.
#undef setpt
#undef ctrl
#undef clk
#undef fb

struct sPID_CONTROLLER
{
  // declare the structure here
  bool   clk_n1;      // clk[n-1]
  double error_n1;    // error[n-1]
  double errorI_n1;   // errorI[n-1]
  double lastT;
  double ctrl_n1;     // ctrl[n-1]
};

extern "C" __declspec(dllexport) void pid_controller_v2(struct sPID_CONTROLLER **opaque, double t, union uData *data)
{
   double  setpt  = data[ 0].d; // input
   bool    clk    = data[ 1].b; // input
   double  fb     = data[ 2].d; // input
   double  Kp     = data[ 3].d; // input parameter
   double  Ki     = data[ 4].d; // input parameter
   double  Kd     = data[ 5].d; // input parameter
   double  Kv     = data[ 6].d; // input parameter
   bool    Itype  = data[ 7].b; // input parameter
   double  maxSat = data[ 8].d; // input parameter
   double  minSat = data[ 9].d; // input parameter
   double &ctrl   = data[10].d; // output

   if(!*opaque)
   {
      *opaque = (struct sPID_CONTROLLER *) malloc(sizeof(struct sPID_CONTROLLER));
      bzero(*opaque, sizeof(struct sPID_CONTROLLER));
      Display("pid_controller: Kp=%f, Ki=%f, Kd=%f, Kv=%f, Integration Method=%s\n", Kp, Ki, Kd, Kv, Itype ? "Trapezoidal" : "Rectangular");
   }
   struct sPID_CONTROLLER *inst = *opaque;

// Implement module evaluation code here:
  if (clk && !inst->clk_n1)   // rising edge
  {
    // time between samples : calculate Tsampling
    double Tsampling =
        t - inst->lastT;   // Tsampling = current time - last rising edge time
    inst->lastT = t;

    // calculate error
    double error = Kv * (setpt - fb);

    // calculate proportional, integral and derivative error
    double errorP = error;

    // For integral anti-windup - Clamping method
    double errorI;
    if ((inst->ctrl_n1 > maxSat || inst->ctrl_n1 < minSat) && (error*inst->ctrl_n1 >= 0)) { // if previous ctrl output is saturated AND current error direction is the same as previous ctrl output direction, then don't accumulate errorI (integral error)
      errorI = inst->errorI_n1;
    } else { // else, accumulate errorI as usual
      if (Itype)   // trapezoidal rule
        errorI = (error + inst->error_n1) * Tsampling / 2 + inst->errorI_n1;
      else   // rectangular method
        errorI = error * Tsampling + inst->errorI_n1;
    }

    double errorD = (error - inst->error_n1) / Tsampling;

    // PID formula
    ctrl = Kp * errorP + Ki * errorI + Kd * errorD;

    // store [n-1] sampling
    inst->error_n1  = error;    // error[n-1] = error[n]
    inst->errorI_n1 = errorI;   // errorI[n-1] = errorI[n]
    inst->ctrl_n1 = ctrl;       // ctrl[n-1] = ctrl[n]
  }

  // save clock state
  inst->clk_n1 = clk;
}

extern "C" __declspec(dllexport) void Trunc(struct sPID_CONTROLLER *inst, double t, union uData *data, double *timestep)
{ // limit the timestep to a tolerance if the circuit causes a change in struct sPID_CONTROLLER
  const double ttol = 1e-9;
  bool    clk    = data[ 1].b; // input

  // if not rising clock edge, we're done
  if (clk == inst->clk_n1 || !clk) { return; }

  *timestep = ttol;
}

extern "C" __declspec(dllexport) void Destroy(struct sPID_CONTROLLER *inst)
{
   free(inst);
}
/*==============================================================================
 * End of pid_controller.cpp
 *============================================================================*/
