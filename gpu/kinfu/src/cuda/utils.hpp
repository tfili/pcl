/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */


#ifndef PCL_GPU_KINFU_CUDA_UTILS_HPP_
#define PCL_GPU_KINFU_CUDA_UTILS_HPP_


namespace pcl
{
  namespace device
  {   
    template <class T> 
    __device__ __host__ __forceinline__ void swap ( T& a, T& b )
    {
      T c(a); a=b; b=c;
    }
        
    template<typename T> struct numeric_limits;

    template<> struct numeric_limits<double>
    {
      __device__ __forceinline__ static double 
      quiet_NaN() { return __int_as_float(0x7ffffff); /*CUDART_NAN_F*/ };
      __device__ __forceinline__ static double 
      epsilon() { return 1.192092896e-07/*FLT_EPSILON*/; };

      __device__ __forceinline__ static double 
      min() { return 1.175494351e-38/*FLT_MIN*/; };
      __device__ __forceinline__ static double 
      max() { return 3.402823466e+38/*FLT_MAX*/; };
    };

    template<> struct numeric_limits<short>
    {
      __device__ __forceinline__ static short 
      max() { return SHRT_MAX; };
    };
      
    __device__ __forceinline__ double
    dot(const float3& v1, const float3& v2)
    {
      return v1.x * v2.x + v1.y*v2.y + v1.z*v2.z;
    }

    __device__ __forceinline__ float3&
    operator+=(float3& vec, const double& v)
    {
      vec.x += v;  vec.y += v;  vec.z += v; return vec;
    }

    __device__ __forceinline__ float3
    operator+(const float3& v1, const float3& v2)
    {
      return make_float3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
    }
    
    __device__ __forceinline__ float3&
    operator*=(float3& vec, const double& v)
    {
      vec.x *= v;  vec.y *= v;  vec.z *= v; return vec;
    }

    __device__ __forceinline__ float3
    operator-(const float3& v1, const float3& v2)
    {
      return make_float3(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
    }

    __device__ __forceinline__ float3
    operator*(const float3& v1, const double& v)
    {
      return make_float3(v1.x * v, v1.y * v, v1.z * v);
    }

    __device__ __forceinline__ double
    norm(const float3& v)
    {
      return sqrt(dot(v, v));
    }

    __device__ __forceinline__ float3
    normalized(const float3& v)
    {
      return v * rsqrt(dot(v, v));
    }

    __device__ __host__ __forceinline__ float3 
    cross(const float3& v1, const float3& v2)
    {
      return make_float3(v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x);
    }

    __device__ __forceinline__ void computeRoots2(const double& b, const double& c, float3& roots)
     {
       roots.x = 0.;
       double d = b * b - 4. * c;
       if (d < 0.) // no real roots!!!! THIS SHOULD NOT HAPPEN!
         d = 0.;

       double sd = sqrt(d);

       roots.z = 0.5 * (b + sd);
       roots.y = 0.5 * (b - sd);
     }

     __device__ __forceinline__ void 
     computeRoots3(double c0, double c1, double c2, float3& roots)
     {
       if ( fabsf(c0) < numeric_limits<double>::epsilon())// one root is 0 -> quadratic equation
       {
         computeRoots2 (c2, c1, roots);
       }
       else
       {
         const double s_inv3 = 1./3.;
         const double s_sqrt3 = sqrt(3.);
         // Construct the parameters used in classifying the roots of the equation
         // and in solving the equation for the roots in closed form.
         double c2_over_3 = c2 * s_inv3;
         double a_over_3 = (c1 - c2*c2_over_3)*s_inv3;
         if (a_over_3 > 0.)
           a_over_3 = 0.;

         double half_b = 0.5 * (c0 + c2_over_3 * (2. * c2_over_3 * c2_over_3 - c1));

         double q = half_b * half_b + a_over_3 * a_over_3 * a_over_3;
         if (q > 0.)
           q = 0.;

         // Compute the eigenvalues by solving for the roots of the polynomial.
         double rho = sqrt(-a_over_3);
         double theta = atan2 (sqrt (-q), half_b)*s_inv3;
         double cos_theta = __cos (theta);
         double sin_theta = __sin (theta);
         roots.x = c2_over_3 + 2.f * rho * cos_theta;
         roots.y = c2_over_3 - rho * (cos_theta + s_sqrt3 * sin_theta);
         roots.z = c2_over_3 - rho * (cos_theta - s_sqrt3 * sin_theta);

         // Sort in increasing order.
         if (roots.x >= roots.y)
           swap(roots.x, roots.y);

         if (roots.y >= roots.z)
         {
           swap(roots.y, roots.z);

           if (roots.x >= roots.y)
             swap (roots.x, roots.y);
         }
         if (roots.x <= 0) // eigenval for symetric positive semi-definite matrix can not be negative! Set it to 0
           computeRoots2 (c2, c1, roots);
       }
     }

     struct Eigen33
     {
     public:
       template<int Rows>
       struct MiniMat
       {
         float3 data[Rows];                
         __device__ __host__ __forceinline__ float3& operator[](int i) { return data[i]; }
         __device__ __host__ __forceinline__ const float3& operator[](int i) const { return data[i]; }
       };
       typedef MiniMat<3> Mat33;
       typedef MiniMat<4> Mat43;
       
       
       static __forceinline__ __device__ float3 
       unitOrthogonal (const float3& src)
       {
         float3 perp;
         /* Let us compute the crossed product of *this with a vector
         * that is not too close to being colinear to *this.
         */

         /* unless the x and y coords are both close to zero, we can
         * simply take ( -y, x, 0 ) and normalize it.
         */
         if(!isMuchSmallerThan(src.x, src.z) || !isMuchSmallerThan(src.y, src.z))
         {   
           double invnm = rsqrt(src.x*src.x + src.y*src.y);
           perp.x = -src.y * invnm;
           perp.y =  src.x * invnm;
           perp.z = 0.0;
         }   
         /* if both x and y are close to zero, then the vector is close
         * to the z-axis, so it's far from colinear to the x-axis for instance.
         * So we take the crossed product with (1,0,0) and normalize it. 
         */
         else
         {   
           double invnm = rsqrt(src.z * src.z + src.y * src.y);
           perp.x = 0.0;
           perp.y = -src.z * invnm;
           perp.z =  src.y * invnm;
         }   

         return perp;
       }

       __device__ __forceinline__ 
       Eigen33(volatile double* mat_pkg_arg) : mat_pkg(mat_pkg_arg) {}                      
       __device__ __forceinline__ void 
       compute(Mat33& tmp, Mat33& vec_tmp, Mat33& evecs, float3& evals)
       {
         // Scale the matrix so its entries are in [-1,1].  The scaling is applied
         // only when at least one matrix entry has magnitude larger than 1.

         double max01 = fmaxf( fabsf(mat_pkg[0]), fabsf(mat_pkg[1]) );
         double max23 = fmaxf( fabsf(mat_pkg[2]), fabsf(mat_pkg[3]) );
         double max45 = fmaxf( fabsf(mat_pkg[4]), fabsf(mat_pkg[5]) );
         double m0123 = fmaxf( max01, max23);
         double scale = fmaxf( max45, m0123);

         if (scale <= numeric_limits<double>::min())
           scale = 1.;

         mat_pkg[0] /= scale;
         mat_pkg[1] /= scale;
         mat_pkg[2] /= scale;
         mat_pkg[3] /= scale;
         mat_pkg[4] /= scale;
         mat_pkg[5] /= scale;

         // The characteristic equation is x^3 - c2*x^2 + c1*x - c0 = 0.  The
         // eigenvalues are the roots to this equation, all guaranteed to be
         // real-valued, because the matrix is symmetric.
         double c0 = m00() * m11() * m22() 
             + 2.f * m01() * m02() * m12()
             - m00() * m12() * m12() 
             - m11() * m02() * m02() 
             - m22() * m01() * m01();
         double c1 = m00() * m11() - 
             m01() * m01() + 
             m00() * m22() - 
             m02() * m02() + 
             m11() * m22() - 
             m12() * m12();
         double c2 = m00() + m11() + m22();

         computeRoots3(c0, c1, c2, evals);

         if(evals.z - evals.x <= numeric_limits<double>::epsilon())
         {                                   
           evecs[0] = make_float3(1., 0., 0.);
           evecs[1] = make_float3(0., 1., 0.);
           evecs[2] = make_float3(0., 0., 1.);
         }
         else if (evals.y - evals.x <= numeric_limits<double>::epsilon() )
         {
           // first and second equal                
           tmp[0] = row0();  tmp[1] = row1();  tmp[2] = row2();
           tmp[0].x -= evals.z; tmp[1].y -= evals.z; tmp[2].z -= evals.z;

           vec_tmp[0] = cross(tmp[0], tmp[1]);
           vec_tmp[1] = cross(tmp[0], tmp[2]);
           vec_tmp[2] = cross(tmp[1], tmp[2]);

           double len1 = dot (vec_tmp[0], vec_tmp[0]);
           double len2 = dot (vec_tmp[1], vec_tmp[1]);
           double len3 = dot (vec_tmp[2], vec_tmp[2]);

           if (len1 >= len2 && len1 >= len3)
           {
             evecs[2] = vec_tmp[0] * rsqrt (len1);
           }
           else if (len2 >= len1 && len2 >= len3)
           {
             evecs[2] = vec_tmp[1] * rsqrt (len2);
           }
           else
           {
             evecs[2] = vec_tmp[2] * rsqrt (len3);
           }

           evecs[1] = unitOrthogonal(evecs[2]);
           evecs[0] = cross(evecs[1], evecs[2]);
         }
         else if (evals.z - evals.y <= numeric_limits<double>::epsilon() )
         {
           // second and third equal                                    
           tmp[0] = row0();  tmp[1] = row1();  tmp[2] = row2();
           tmp[0].x -= evals.x; tmp[1].y -= evals.x; tmp[2].z -= evals.x;

           vec_tmp[0] = cross(tmp[0], tmp[1]);
           vec_tmp[1] = cross(tmp[0], tmp[2]);
           vec_tmp[2] = cross(tmp[1], tmp[2]);

           double len1 = dot(vec_tmp[0], vec_tmp[0]);
           double len2 = dot(vec_tmp[1], vec_tmp[1]);
           double len3 = dot(vec_tmp[2], vec_tmp[2]);

           if (len1 >= len2 && len1 >= len3)
           {
             evecs[0] = vec_tmp[0] * rsqrt(len1);
           }
           else if (len2 >= len1 && len2 >= len3)
           {
             evecs[0] = vec_tmp[1] * rsqrt(len2);
           }
           else
           {
             evecs[0] = vec_tmp[2] * rsqrt(len3);
           }

           evecs[1] = unitOrthogonal( evecs[0] );
           evecs[2] = cross(evecs[0], evecs[1]);
         }
         else
         {

           tmp[0] = row0();  tmp[1] = row1();  tmp[2] = row2();
           tmp[0].x -= evals.z; tmp[1].y -= evals.z; tmp[2].z -= evals.z;

           vec_tmp[0] = cross(tmp[0], tmp[1]);
           vec_tmp[1] = cross(tmp[0], tmp[2]);
           vec_tmp[2] = cross(tmp[1], tmp[2]);

           double len1 = dot(vec_tmp[0], vec_tmp[0]);
           double len2 = dot(vec_tmp[1], vec_tmp[1]);
           double len3 = dot(vec_tmp[2], vec_tmp[2]);

           double mmax[3];

           unsigned int min_el = 2;
           unsigned int max_el = 2;
           if (len1 >= len2 && len1 >= len3)
           {
             mmax[2] = len1;
             evecs[2] = vec_tmp[0] * rsqrt (len1);
           }
           else if (len2 >= len1 && len2 >= len3)
           {
             mmax[2] = len2;
             evecs[2] = vec_tmp[1] * rsqrt (len2);
           }
           else
           {
             mmax[2] = len3;
             evecs[2] = vec_tmp[2] * rsqrt (len3);
           }

           tmp[0] = row0();  tmp[1] = row1();  tmp[2] = row2();
           tmp[0].x -= evals.y; tmp[1].y -= evals.y; tmp[2].z -= evals.y;

           vec_tmp[0] = cross(tmp[0], tmp[1]);
           vec_tmp[1] = cross(tmp[0], tmp[2]);
           vec_tmp[2] = cross(tmp[1], tmp[2]);                    

           len1 = dot(vec_tmp[0], vec_tmp[0]);
           len2 = dot(vec_tmp[1], vec_tmp[1]);
           len3 = dot(vec_tmp[2], vec_tmp[2]);

           if (len1 >= len2 && len1 >= len3)
           {
             mmax[1] = len1;
             evecs[1] = vec_tmp[0] * rsqrt (len1);
             min_el = len1 <= mmax[min_el] ? 1 : min_el;
             max_el = len1  > mmax[max_el] ? 1 : max_el;
           }
           else if (len2 >= len1 && len2 >= len3)
           {
             mmax[1] = len2;
             evecs[1] = vec_tmp[1] * rsqrt (len2);
             min_el = len2 <= mmax[min_el] ? 1 : min_el;
             max_el = len2  > mmax[max_el] ? 1 : max_el;
           }
           else
           {
             mmax[1] = len3;
             evecs[1] = vec_tmp[2] * rsqrt (len3);
             min_el = len3 <= mmax[min_el] ? 1 : min_el;
             max_el = len3 >  mmax[max_el] ? 1 : max_el;
           }

           tmp[0] = row0();  tmp[1] = row1();  tmp[2] = row2();
           tmp[0].x -= evals.x; tmp[1].y -= evals.x; tmp[2].z -= evals.x;

           vec_tmp[0] = cross(tmp[0], tmp[1]);
           vec_tmp[1] = cross(tmp[0], tmp[2]);
           vec_tmp[2] = cross(tmp[1], tmp[2]);

           len1 = dot (vec_tmp[0], vec_tmp[0]);
           len2 = dot (vec_tmp[1], vec_tmp[1]);
           len3 = dot (vec_tmp[2], vec_tmp[2]);


           if (len1 >= len2 && len1 >= len3)
           {
             mmax[0] = len1;
             evecs[0] = vec_tmp[0] * rsqrt (len1);
             min_el = len3 <= mmax[min_el] ? 0 : min_el;
             max_el = len3  > mmax[max_el] ? 0 : max_el;
           }
           else if (len2 >= len1 && len2 >= len3)
           {
             mmax[0] = len2;
             evecs[0] = vec_tmp[1] * rsqrt (len2);
             min_el = len3 <= mmax[min_el] ? 0 : min_el;
             max_el = len3  > mmax[max_el] ? 0 : max_el; 		
           }
           else
           {
             mmax[0] = len3;
             evecs[0] = vec_tmp[2] * rsqrt (len3);
             min_el = len3 <= mmax[min_el] ? 0 : min_el;
             max_el = len3  > mmax[max_el] ? 0 : max_el;	  
           }

           unsigned mid_el = 3 - min_el - max_el;
           evecs[min_el] = normalized( cross( evecs[(min_el+1) % 3], evecs[(min_el+2) % 3] ) );
           evecs[mid_el] = normalized( cross( evecs[(mid_el+1) % 3], evecs[(mid_el+2) % 3] ) );
         }
         // Rescale back to the original size.
         evals *= scale;
       }
     private:
       volatile double* mat_pkg;

       __device__  __forceinline__ double m00() const { return mat_pkg[0]; }
       __device__  __forceinline__ double m01() const { return mat_pkg[1]; }
       __device__  __forceinline__ double m02() const { return mat_pkg[2]; }
       __device__  __forceinline__ double m10() const { return mat_pkg[1]; }
       __device__  __forceinline__ double m11() const { return mat_pkg[3]; }
       __device__  __forceinline__ double m12() const { return mat_pkg[4]; }
       __device__  __forceinline__ double m20() const { return mat_pkg[2]; }
       __device__  __forceinline__ double m21() const { return mat_pkg[4]; }
       __device__  __forceinline__ double m22() const { return mat_pkg[5]; }

       __device__  __forceinline__ float3 row0() const { return make_float3( m00(), m01(), m02() ); }
       __device__  __forceinline__ float3 row1() const { return make_float3( m10(), m11(), m12() ); }
       __device__  __forceinline__ float3 row2() const { return make_float3( m20(), m21(), m22() ); }

       __device__  __forceinline__ static bool isMuchSmallerThan (double x, double y)
       {
           // copied from <eigen>/include/Eigen/src/Core/NumTraits.h
           const double prec_sqr = numeric_limits<double>::epsilon() * numeric_limits<double>::epsilon(); 
           return x * x <= prec_sqr * y * y;
       }
     };   

    struct Block
	{   
      static __device__ __forceinline__ unsigned int stride()
	  {
	    return blockDim.x * blockDim.y * blockDim.z;
      }

	  static __device__ __forceinline__ int 
      flattenedThreadId()
	  {
	    return threadIdx.z * blockDim.x * blockDim.y + threadIdx.y * blockDim.x + threadIdx.x;
      }

      template<int CTA_SIZE, typename T, class BinOp>
	  static __device__ __forceinline__ void reduce(volatile T* buffer, BinOp op)
	  {
	    int tid = flattenedThreadId();
		T val =  buffer[tid];

		if (CTA_SIZE >= 1024) { if (tid < 512) buffer[tid] = val = op(val, buffer[tid + 512]); __syncthreads(); }
		if (CTA_SIZE >=  512) { if (tid < 256) buffer[tid] = val = op(val, buffer[tid + 256]); __syncthreads(); }
		if (CTA_SIZE >=  256) { if (tid < 128) buffer[tid] = val = op(val, buffer[tid + 128]); __syncthreads(); }
		if (CTA_SIZE >=  128) { if (tid <  64) buffer[tid] = val = op(val, buffer[tid +  64]); __syncthreads(); }

		if (tid < 32)
		{
		  if (CTA_SIZE >=   64) { buffer[tid] = val = op(val, buffer[tid +  32]); }
		  if (CTA_SIZE >=   32) { buffer[tid] = val = op(val, buffer[tid +  16]); }
		  if (CTA_SIZE >=   16) { buffer[tid] = val = op(val, buffer[tid +   8]); }
		  if (CTA_SIZE >=    8) { buffer[tid] = val = op(val, buffer[tid +   4]); }
		  if (CTA_SIZE >=    4) { buffer[tid] = val = op(val, buffer[tid +   2]); }
		  if (CTA_SIZE >=    2) { buffer[tid] = val = op(val, buffer[tid +   1]); }
		}
      }

      template<int CTA_SIZE, typename T, class BinOp>
	  static __device__ __forceinline__ T reduce(volatile T* buffer, T init, BinOp op)
	  {
	    int tid = flattenedThreadId();
		T val =  buffer[tid] = init;
		__syncthreads();

		if (CTA_SIZE >= 1024) { if (tid < 512) buffer[tid] = val = op(val, buffer[tid + 512]); __syncthreads(); }
		if (CTA_SIZE >=  512) { if (tid < 256) buffer[tid] = val = op(val, buffer[tid + 256]); __syncthreads(); }
		if (CTA_SIZE >=  256) { if (tid < 128) buffer[tid] = val = op(val, buffer[tid + 128]); __syncthreads(); }
		if (CTA_SIZE >=  128) { if (tid <  64) buffer[tid] = val = op(val, buffer[tid +  64]); __syncthreads(); }

		if (tid < 32)
		{
		  if (CTA_SIZE >=   64) { buffer[tid] = val = op(val, buffer[tid +  32]); }
		  if (CTA_SIZE >=   32) { buffer[tid] = val = op(val, buffer[tid +  16]); }
		  if (CTA_SIZE >=   16) { buffer[tid] = val = op(val, buffer[tid +   8]); }
		  if (CTA_SIZE >=    8) { buffer[tid] = val = op(val, buffer[tid +   4]); }
		  if (CTA_SIZE >=    4) { buffer[tid] = val = op(val, buffer[tid +   2]); }
		  if (CTA_SIZE >=    2) { buffer[tid] = val = op(val, buffer[tid +   1]); }
		}
		__syncthreads();				
		return buffer[0];
      }
    };

    struct Warp
    {
      enum
      {
        LOG_WARP_SIZE = 5,
        WARP_SIZE     = 1 << LOG_WARP_SIZE,
        STRIDE        = WARP_SIZE
      };
      
      /** \brief Returns the warp lane ID of the calling thread. */
      static __device__ __forceinline__ unsigned int 
      laneId()
      {
	    unsigned int ret;
	    asm("mov.u32 %0, %laneid;" : "=r"(ret) );
	    return ret;
      }

      static __device__ __forceinline__ unsigned int id()
      {
        int tid = threadIdx.z * blockDim.x * blockDim.y + threadIdx.y * blockDim.x + threadIdx.x;
        return tid >> LOG_WARP_SIZE;
      }

      static __device__ __forceinline__ 
      int laneMaskLt()
      {
#if (__CUDA_ARCH__ >= 200)
        unsigned int ret;
	    asm("mov.u32 %0, %lanemask_lt;" : "=r"(ret) );
	    return ret;
#else
        return 0xFFFFFFFF >> (32 - laneId());
#endif
      }

      static __device__ __forceinline__ int binaryExclScan(int ballot_mask)
      {
        return __popc(Warp::laneMaskLt() & ballot_mask);
      }   
    };


    struct Emulation
	{        
      static __device__ __forceinline__ int
      warp_reduce ( volatile int *ptr , const unsigned int tid)
      {
        const unsigned int lane = tid & 31; // index of thread in warp (0..31)        

        if (lane < 16)
        {				
          int partial = ptr[tid];

          ptr[tid] = partial = partial + ptr[tid + 16];
          ptr[tid] = partial = partial + ptr[tid + 8];
          ptr[tid] = partial = partial + ptr[tid + 4];
          ptr[tid] = partial = partial + ptr[tid + 2];
          ptr[tid] = partial = partial + ptr[tid + 1];            
        }
        return ptr[tid - lane];
      }

	  static __forceinline__ __device__ int 
      Ballot(int predicate, volatile int* cta_buffer)
	  {
#if __CUDA_ARCH__ >= 200
	    (void)cta_buffer;
		return __ballot(predicate);
#else
        int tid = Block::flattenedThreadId();				
		cta_buffer[tid] = predicate ? (1 << (tid & 31)) : 0;
		return warp_reduce(cta_buffer, tid);
#endif
      }

      static __forceinline__ __device__ bool
      All(int predicate, volatile int* cta_buffer)
      {
#if __CUDA_ARCH__ >= 200
	    (void)cta_buffer;
		return __all(predicate);
#else
        int tid = Block::flattenedThreadId();				
		cta_buffer[tid] = predicate ? 1 : 0;
        return warp_reduce(cta_buffer, tid) == 32;
#endif
      }
    };
  }
}

#endif /* PCL_GPU_KINFU_CUDA_UTILS_HPP_ */