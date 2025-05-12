// Automatically generated C++ file on Mon Sep  2 00:56:05 2024
//
// To build with Digital Mars C++ Compiler:
//
//    dmc -mn -WD pid_controller_limits.cpp kernel32.lib

#include <malloc.h>

extern "C" __declspec(dllexport) int (*Display)(const char *format, ...) = 0; // works like printf()
extern "C" __declspec(dllexport) const double *DegreesC                  = 0; // pointer to current circuit temperature

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
   double errorD_n1;   // errorD[n-1]
   double lastT;
};

extern "C" __declspec(dllexport) void pid_controller_limits(struct sPID_CONTROLLER **opaque, double t, union uData *data)
{
   double  setpt        = data[ 0].d; // input
   bool    clk          = data[ 1].b; // input
   double  fb           = data[ 2].d; // input
   double  Kp           = data[ 3].d; // input parameter
   double  Ki           = data[ 4].d; // input parameter
   double  Kd           = data[ 5].d; // input parameter
   double  Kv           = data[ 6].d; // input parameter
   bool    Itype        = data[ 7].b; // input parameter
   double  Plimit       = data[ 8].d; // input parameter
   double  Ilimit       = data[ 9].d; // input parameter
   double  Dlimit       = data[10].d; // input parameter
   double  ctrl_limit_h = data[11].d; // input parameter
   double  ctrl_limit_l = data[12].d; // input parameter
   double &ctrl         = data[13].d; // output

   if(!*opaque)
   {
      *opaque = (struct sPID_CONTROLLER *) malloc(sizeof(struct sPID_CONTROLLER));
      bzero(*opaque, sizeof(struct sPID_CONTROLLER));

      Display("pid_controller: Kp=%f, Ki=%f, Kd=%f, Kv=%f, Integration Method=%s\n",Kp, Ki, Kd, Kv, Itype ? "Trapezoidal" : "Rectangular");
   }
   struct sPID_CONTROLLER *inst = *opaque;

   // Implement module evaluation code here:
   if (clk && !inst->clk_n1)   // rising edge
   {
      // time between samples : calculate Tsampling
      double Tsampling = t - inst->lastT;   // Tsampling = current time - last rising edge time
      inst->lastT      = t;

      // calculate error
      double error = Kv * (setpt - fb); //altered to make limits work correctly

      // calculate proportional, integral and derivative error
      double errorP;
      double errorP1=Kp*error;

      if (errorP1>Plimit)
      {
         errorP=Plimit;
      }
      else if(errorP1<-Plimit)
      {
         errorP=-Plimit;
      }
      else
   {
         errorP = errorP1;
      }

      double errorI;

      if (Itype)   // trapezoidal rule
      {
         double errorI1 = Ki*((error + inst->error_n1) * Tsampling / 2 + inst->errorI_n1); //altered to make limits work correctly

         if (errorI1>Ilimit)
         {
            errorI=Ilimit;
         }
         else if(errorI1<-Ilimit)
         {
            errorI=-Ilimit;
         }
         else
      {
            errorI = errorI1;
         }

      }
      else   // rectangular method
   {

         double errorI1 = Ki*(error * Tsampling + inst->errorI_n1); //altered to make limits work correctly

         if (errorI1>Ilimit)
         {
            errorI=Ilimit;
         }
         else if(errorI1<-Ilimit)
         {
            errorI=-Ilimit;
         }
         else
      {
            errorI = errorI1;
         }

      }



      double errorD1 = Kd*((error - inst->error_n1) / Tsampling); // matlab : matched. Altered to make limits work correctly
      double errorD;

      //double errorD1 = Kd*((error - inst->error_n1) / Tsampling * 2 - inst->errorD_n1);   // matlab : tustin
      //double errorD1 = Kd*(50*(error - inst->error_n1) + (1-50*Tsampling)*inst->errorD_n1);   // Forward Euler
      //double errorD1 = Kd*(1/(1+1e6*Tsampling)*(1e6*(error - inst->error_n1) + inst->errorD_n1);  // Backward Euler
      //double errorD1 = Kd*((1/(1+100*Tsampling/2)*(100*(error - inst->error_n1)) + (1-100*Tsampling/2)*inst->errorD_n1));  // Trapezoidal Derivative
      if (errorD1>Dlimit)
      {
         errorD=Dlimit;
      }
      else if(errorD1<-Dlimit)
      {
         errorD=-Dlimit;
      }
      else
   {
         errorD = errorD1;
      }

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

      if (Ki==0)                          //regulltor would malfunction in case of Kd or Ki == 0. Altered to store n-1 error without K multiplicator
      {
         inst->errorI_n1 = 0;
      }
      else
   {
         inst->errorI_n1 = errorI/Ki; // errorI[n-1] = errorI[n]
      }

      if (Kd==0)                          //regulltor would malfunction in case of Kd or Ki == 0. Altered to store n-1 error without K multiplicator
      {
         inst->errorD_n1 = 0;
      }
      else
   {
         inst->errorD_n1 = errorD/Kd; // errorD[n-1] = errorD[n]
      }

   }

   // save clock state
   inst->clk_n1 = clk;
}

extern "C" __declspec(dllexport) double MaxExtStepSize(struct sPID_CONTROLLER *inst)
{
   return 1e308; // implement a good choice of max timestep size that depends on struct sPID_CONTROLLER
}

extern "C" __declspec(dllexport) void Trunc(struct sPID_CONTROLLER *inst, double t, union uData *data, double *timestep)
{ // limit the timestep to a tolerance if the circuit causes a change in struct sPID_CONTROLLER
   const double ttol = 10e-12; // 10ps default tolerance
   if(*timestep > ttol)
   {
      struct sPID_CONTROLLER tmp = *inst;
      pid_controller_limits(&(&tmp), t, data);

      if(tmp.clk_n1 != inst->clk_n1 & !inst->clk_n1)  // clock rising edge reduce timestep to TTOL
         *timestep = ttol;
   }
}

extern "C" __declspec(dllexport) void Destroy(struct sPID_CONTROLLER *inst)
{
   free(inst);
}
