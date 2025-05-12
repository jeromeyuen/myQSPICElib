// Automatically generated C++ file on Mon May 12 15:44:51 2025
//
// To build with Digital Mars C++ Compiler:
//
//    dmc -mn -WD pid_controller_limits_bc.cpp kernel32.lib

/*==============================================================================
 * Revision History:
 * 2025.03.26 -- Initial release by Farel, modified from KSKelvin and RDunn.
 * 2025.05.12 -- Modified by JY with anti-windup via back calculation.
 *============================================================================*/

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
#undef P_error
#undef I_error
#undef ctrl_sum
#undef D_error
#undef sat

struct sPID_CONTROLLER_LIMITS_BC
{
  // declare the structure here
   bool   clk_n1;      // clk[n-1]
   double error_n1;    // error[n-1]
   double errorItmp_n1; // (JY) errorItmp[n-1]
   double errorI_n1;   // errorI[n-1]
   double errorD_n1;   // errorD[n-1]
   double lastT;
   double ctrl_error;  // (JY) ctrl[n-1] - ctrl1[n-1], i.e. the difference in the control error from the previous timestep, used for anti-windup compensation.
};

extern "C" __declspec(dllexport) void pid_controller_limits_bc(struct sPID_CONTROLLER_LIMITS_BC **opaque, double t, union uData *data)
{
   double  setpt        = data[ 0].d; // input
   bool    clk          = data[ 1].b; // input
   double  fb           = data[ 2].d; // input
   double  Kp           = data[ 3].d; // input parameter
   double  Ki           = data[ 4].d; // input parameter
   double  Kd           = data[ 5].d; // input parameter
   double  Kv           = data[ 6].d; // input parameter
   bool    Itype        = data[ 7].b; // input parameter
   double  ctrl_limit_h = data[ 8].d; // input parameter
   double  ctrl_limit_l = data[ 9].d; // input parameter
   double  Kb           = data[10].d; // input parameter
   double &ctrl         = data[11].d; // output
   double &P_error      = data[12].d; // output
   double &I_error      = data[13].d; // output
   double &ctrl_sum     = data[14].d; // output
   double &D_error      = data[15].d; // output
   double &sat          = data[16].d; // output

   if(!*opaque)
   {
      *opaque = (struct sPID_CONTROLLER_LIMITS_BC *) malloc(sizeof(struct sPID_CONTROLLER_LIMITS_BC));
      bzero(*opaque, sizeof(struct sPID_CONTROLLER_LIMITS_BC));

      Display("pid_controller: Kp=%f, Ki=%f, Kd=%f, Kv=%f, Integration Method=%s\n",Kp, Ki, Kd, Kv, Itype ? "Trapezoidal" : "Rectangular");
   }
   struct sPID_CONTROLLER_LIMITS_BC *inst = *opaque;

// Implement module evaluation code here:
   if (clk && !inst->clk_n1)   // rising edge
   {
      // time between samples : calculate Tsampling
      double Tsampling = t - inst->lastT;   // Tsampling = current time - last rising edge time
      inst->lastT      = t;

      // calculate error
      double error = Kv * (setpt - fb);

      // calculate proportional, integral and derivative error
      double errorP=Kp*error;

      double errorItmp = error + (Kb * inst->ctrl_error); // (JY) adding back calculation term to error term for integration gain.
      double errorI;

      if (Itype)   // trapezoidal rule
      {
        errorI = Ki*((errorItmp + inst->errorItmp_n1) * Tsampling / 2 + inst->errorI_n1);
      }
      else   // rectangular method
   {
        errorI = Ki*(errorItmp * Tsampling + inst->errorI_n1);
      }


      double errorD = Kd*((error - inst->error_n1) / Tsampling); // matlab : matched.

      //double errorD1 = Kd*((error - inst->error_n1) / Tsampling * 2 - inst->errorD_n1);   // matlab : tustin
      //double errorD1 = Kd*(50*(error - inst->error_n1) + (1-50*Tsampling)*inst->errorD_n1);   // Forward Euler
      //double errorD1 = Kd*(1/(1+1e6*Tsampling)*(1e6*(error - inst->error_n1) + inst->errorD_n1);  // Backward Euler
      //double errorD1 = Kd*((1/(1+100*Tsampling/2)*(100*(error - inst->error_n1)) + (1-100*Tsampling/2)*inst->errorD_n1));  // Trapezoidal Derivative


      // PID formula
      double ctrl1 = errorP + errorI + errorD; //altered to make limits work correctly

      if (ctrl1>ctrl_limit_h)
      {
         ctrl=ctrl_limit_h;
      }
      else if(ctrl1<ctrl_limit_l)
      {
         ctrl=ctrl_limit_l;
      }
      else
   {
         ctrl = ctrl1;
      }


      // store [n-1] sampling
      inst->error_n1  = error;    // error[n-1] = error[n]
      inst->errorItmp_n1 = errorItmp; // (JY) errorItmp[n-1] = errorItmp[n]
      inst->ctrl_error = ctrl - ctrl1; // (JY) ctrl[n-1] - ctrl1[n-1]

      if (Ki==0)                          //regulltor would malfunction in case of Kd or Ki == 0. Altered to store n-1 error without K multiplicator
      {
         inst->errorI_n1 = 0;
      }
      else
   {
         inst->errorI_n1 = errorI/Ki; // errorI[n-1] = errorI[n]
         // (JY) Modified 'errorI/Ki' to just 'errorI', since we are using 'inst->errorI_n1' in the expression where 'errorI' has already been scaled up by 'Ki'.
      }

      if (Kd==0)                          //regulltor would malfunction in case of Kd or Ki == 0. Altered to store n-1 error without K multiplicator
      {
         inst->errorD_n1 = 0;
      }
      else
   {
         inst->errorD_n1 = errorD/Kd; // errorD[n-1] = errorD[n]
      }

      // (JY) exposing internal values to the C++ block:
      P_error = errorP;
      I_error = errorI;
      D_error = errorD;
      ctrl_sum = ctrl1;
      sat = inst->ctrl_error;
   }

   // save clock state
   inst->clk_n1 = clk;
}


extern "C" __declspec(dllexport) double MaxExtStepSize(struct sPID_CONTROLLER_LIMITS_BC *inst, double t)
{
   return 1e308; // implement a good choice of max timestep size that depends on struct sPID_CONTROLLER_LIMITS_BC
}

extern "C" __declspec(dllexport) void Trunc(struct sPID_CONTROLLER_LIMITS_BC *inst, double t, union uData *data, double *timestep)
{ // limit the timestep to a tolerance if the circuit causes a change in struct sPID_CONTROLLER_LIMITS_BC
   const double ttol = 10e-12; // 10ps default tolerance
   if(*timestep > ttol)
   {
      struct sPID_CONTROLLER_LIMITS_BC tmp = *inst;
      pid_controller_limits_bc(&(&tmp), t, data);

      bool    clk          = data[ 1].b; // input

      // // if not rising clock edge, we're done
      if (clk == inst->clk_n1 || !clk) { return; }

      *timestep = ttol;
   }
}

extern "C" __declspec(dllexport) void Destroy(struct sPID_CONTROLLER_LIMITS_BC *inst)
{
   free(inst);
}
