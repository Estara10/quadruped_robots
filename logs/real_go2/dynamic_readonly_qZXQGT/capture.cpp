#include <librealsense2/rs.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cstring>
#include <string>
#include <iomanip>
#include <librealsense2/rsutil.h>

int main(int argc,char**argv) {
 try {
  std::cout << std::setprecision(17); rs2::context ctx; auto devices=ctx.query_devices();
  if(devices.size()!=1) throw std::runtime_error("Expected exactly one RealSense");
  auto dev=devices[0]; std::string serial=dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
  std::cout<<"serial="<<serial<<" firmware="<<dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION)<<std::endl;
  auto sensor=dev.first<rs2::depth_sensor>();
  if(argc==1){
   for(auto p:sensor.get_stream_profiles()) if(p.stream_type()==RS2_STREAM_DEPTH && p.format()==RS2_FORMAT_Z16){auto v=p.as<rs2::video_stream_profile>();std::cout<<v.width()<<" "<<v.height()<<" "<<p.fps()<<std::endl;}
   return 0;
  }
  if(argc!=5) throw std::runtime_error("width height fps output_directory");
  int w=std::stoi(argv[1]), h=std::stoi(argv[2]), fps=std::stoi(argv[3]);std::string out=argv[4];
  if(serial!="238322070866") throw std::runtime_error("Unexpected camera serial");
  std::vector<rs2_option> opts={RS2_OPTION_EMITTER_ENABLED,RS2_OPTION_ENABLE_AUTO_EXPOSURE,RS2_OPTION_LASER_POWER};
  std::vector<float> original;for(auto o:opts) original.push_back(sensor.get_option(o));
  rs2::pipeline pipe(ctx); bool running=false;
  auto restore=[&](){for(size_t i=0;i<opts.size();++i){sensor.set_option(opts[i],original[i]);std::cout<<"restored "<<rs2_option_to_string(opts[i])<<"="<<sensor.get_option(opts[i])<<std::endl;}};
  try {
   sensor.set_option(opts[0],1);sensor.set_option(opts[1],1);sensor.set_option(opts[2],150);
   rs2::config cfg;cfg.enable_device(serial);cfg.enable_stream(RS2_STREAM_DEPTH,w,h,RS2_FORMAT_Z16,fps);cfg.enable_stream(RS2_STREAM_INFRARED,1,w,h,RS2_FORMAT_Y8,fps);
   auto profile=pipe.start(cfg);running=true;
   auto actual=profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();auto intr=actual.get_intrinsics();
   float scale=profile.get_device().first<rs2::depth_sensor>().get_depth_scale();
   std::ofstream meta(out+"/metadata.txt");meta<<"serial="<<serial<<"\nwidth="<<actual.width()<<"\nheight="<<actual.height()<<"\nfps="<<actual.fps()<<"\nscale="<<scale<<"\nfx="<<intr.fx<<"\nfy="<<intr.fy<<"\nppx="<<intr.ppx<<"\nppy="<<intr.ppy<<"\n";
   for(size_t i=0;i<opts.size();++i)meta<<rs2_option_to_string(opts[i])<<"="<<sensor.get_option(opts[i])<<"\noriginal_"<<rs2_option_to_string(opts[i])<<"="<<original[i]<<"\n";
   if(actual.width()!=w||actual.height()!=h||actual.fps()!=fps)throw std::runtime_error("Profile mismatch");
   auto start=std::chrono::steady_clock::now();while(std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<3)pipe.wait_for_frames(3000);
   rs2::hole_filling_filter hole(1.f); std::ofstream csv(out+"/frames.csv"); csv << std::setprecision(17);csv<<"i,frame_number,device_ms,host_s,zero_fraction,center_zero_fraction,center_median_m,exposure_us,gain,saved,filled_zero_fraction,filled_center_zero_fraction,filled_pixels,original_valid_changed,center_pixel_depth_m\n";
   start=std::chrono::steady_clock::now();
   meta << std::setprecision(17) << "capture_start_steady_s=" << std::chrono::duration<double>(start.time_since_epoch()).count() << "\n" << std::flush;
   unsigned long long previous=0;int count=0;
   while(true){
    auto frames=pipe.wait_for_frames(3000);auto f=frames.get_depth_frame();auto infrared=frames.get_infrared_frame(1);auto number=f.get_frame_number();if(number==previous)continue;previous=number;
    double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    const auto*data=static_cast<const uint16_t*>(f.get_data());int zeros=0,cz=0;std::vector<uint16_t> center;
    for(int y=0;y<h;++y)for(int x=0;x<w;++x){uint16_t d=data[y*w+x];zeros+=(d==0);if(x>=int(w*11/24.0)&&x<int(w*13/24.0)&&y>=int(h*23/54.0)&&y<int(h*31/54.0)){cz+=(d==0);if(d)center.push_back(d);}}
    int roi=(int(w*13/24.0)-int(w*11/24.0))*(int(h*31/54.0)-int(h*23/54.0));double med=-1;if(!center.empty()){std::sort(center.begin(),center.end());med=center[center.size()/2]*scale;}
    auto md=[&](rs2_frame_metadata_value k)->double{return f.supports_frame_metadata(k)?double(f.get_frame_metadata(k)):-1;};
    auto filtered=hole.process(f).as<rs2::depth_frame>();
    auto fd=static_cast<const uint16_t*>(filtered.get_data());int fz=0,fcz=0,filled=0,changed=0;
    for(int y=0;y<h;++y)for(int x=0;x<w;++x){int j=y*w+x;fz+=(fd[j]==0);filled+=(data[j]==0&&fd[j]!=0);changed+=(data[j]!=0&&data[j]!=fd[j]);if(x>=int(w*11/24.0)&&x<int(w*13/24.0)&&y>=int(h*23/54.0)&&y<int(h*31/54.0))fcz+=(fd[j]==0);}
    bool save=true;
    csv<<count<<","<<number<<","<<f.get_timestamp()<<","<<elapsed<<","<<double(zeros)/(w*h)<<","<<double(cz)/roi<<","<<med<<","<<md(RS2_FRAME_METADATA_ACTUAL_EXPOSURE)<<","<<md(RS2_FRAME_METADATA_GAIN_LEVEL)<<","<<save<<","<<double(fz)/(w*h)<<","<<double(fcz)/roi<<","<<filled<<","<<changed<<","<<data[135*w+240]*scale<<"\n";
    if(save){
     if(count%120==0){
     std::ofstream irfile(out+"/"+std::to_string(count)+"_ir1.u8",std::ios::binary);irfile.write(static_cast<const char*>(infrared.get_data()),w*h);
     float pixel[2]={240,135},point[3],ipoint[3],ipixel[2];auto irprofile=infrared.get_profile().as<rs2::video_stream_profile>();auto ii=irprofile.get_intrinsics();auto extr=f.get_profile().get_extrinsics_to(irprofile);
     rs2_deproject_pixel_to_point(point,&intr,pixel,data[135*w+240]*scale);rs2_transform_point_to_point(ipoint,&extr,point);rs2_project_point_to_pixel(ipixel,&ii,ipoint);
     std::ofstream marker(out+"/"+std::to_string(count)+"_marker.txt");marker<<std::setprecision(17)<<ipixel[0]<<" "<<ipixel[1]<<" "<<point[2]<<" "<<f.get_timestamp()<<" "<<infrared.get_timestamp()<<"\n";
     }
     std::ofstream filtered_bin(out+"/"+std::to_string(count)+"_filled.u16",std::ios::binary);filtered_bin.write(reinterpret_cast<const char*>(fd),w*h*2);std::ofstream bin(out+"/"+std::to_string(count)+"_depth.u16",std::ios::binary);bin.write(reinterpret_cast<const char*>(data),w*h*2);}
    ++count; if(count==1){csv.flush(); std::cout<<"CAPTURE_READY"<<std::endl;} if(elapsed>=120 || std::ifstream(out+"/../STOP").good())break;
   }
   pipe.stop();running=false;restore();std::cout<<"complete frames="<<count<<std::endl;
  }catch(...){if(running)pipe.stop();restore();throw;}
  return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 1;}
}
