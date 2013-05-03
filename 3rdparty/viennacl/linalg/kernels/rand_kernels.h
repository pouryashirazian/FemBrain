#ifndef VIENNACL_RAND_KERNELS_HPP_
#define VIENNACL_RAND_KERNELS_HPP_
#include "viennacl/tools/tools.hpp"
#include "viennacl/ocl/kernel.hpp"
#include "viennacl/ocl/platform.hpp"
#include "viennacl/ocl/utils.hpp"
#include "viennacl/linalg/kernels/rand_source.h"

//Automatically generated file from aux-directory, do not edit manually!
/** @file rand_kernels.h
 *  @brief OpenCL kernel file, generated automatically from scripts in auxiliary/. */
namespace viennacl
{
 namespace linalg
 {
  namespace kernels
  {
   template<class TYPE, unsigned int alignment>
   struct rand;


    /////////////// single precision kernels //////////////// 
   template <>
   struct rand<float, 1>
   {
    static std::string program_name()
    {
      return "f_rand_1";
    }
    static void init()
    {
      viennacl::ocl::DOUBLE_PRECISION_CHECKER<float>::apply();
      static std::map<cl_context, bool> init_done;
      viennacl::ocl::context & context_ = viennacl::ocl::current_context();
      if (!init_done[context_.handle().get()])
      {
        std::string source;
        source.append(rand_align1__mwc64x);
        source.append(rand_align1_dump_gaussian);
        source.append(rand_align1_dump_uniform);
        std::string prog_name = program_name();
        #ifdef VIENNACL_BUILD_INFO
        std::cout << "Creating program " << prog_name << std::endl;
        #endif
        context_.add_program(source, prog_name);
        viennacl::ocl::program & prog_ = context_.get_program(prog_name);
        prog_.add_kernel("_mwc64x");
        prog_.add_kernel("dump_gaussian");
        prog_.add_kernel("dump_uniform");
        init_done[context_.handle().get()] = true;
       } //if
     } //init
    }; // struct



    /////////////// double precision kernels //////////////// 
   template <>
   struct rand<double, 1>
   {
    static std::string program_name()
    {
      return "d_rand_1";
    }
    static void init()
    {
      viennacl::ocl::DOUBLE_PRECISION_CHECKER<double>::apply();
      static std::map<cl_context, bool> init_done;
      viennacl::ocl::context & context_ = viennacl::ocl::current_context();
      if (!init_done[context_.handle().get()])
      {
        std::string source;
        std::string fp64_ext = viennacl::ocl::current_device().double_support_extension();
        source.append(viennacl::tools::make_double_kernel(rand_align1__mwc64x, fp64_ext));
        source.append(viennacl::tools::make_double_kernel(rand_align1_dump_gaussian, fp64_ext));
        source.append(viennacl::tools::make_double_kernel(rand_align1_dump_uniform, fp64_ext));
        std::string prog_name = program_name();
        #ifdef VIENNACL_BUILD_INFO
        std::cout << "Creating program " << prog_name << std::endl;
        #endif
        context_.add_program(source, prog_name);
        viennacl::ocl::program & prog_ = context_.get_program(prog_name);
        prog_.add_kernel("_mwc64x");
        prog_.add_kernel("dump_gaussian");
        prog_.add_kernel("dump_uniform");
        init_done[context_.handle().get()] = true;
       } //if
     } //init
    }; // struct


  }  //namespace kernels
 }  //namespace linalg
}  //namespace viennacl
#endif

