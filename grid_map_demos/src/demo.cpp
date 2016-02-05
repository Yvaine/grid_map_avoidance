void initDevice()
{
    //initDevice
	rs::log_to_console(rs::log_severity::warn);
	rs::context ctx;
    if(ctx.get_device_count() == 0) throw std::runtime_error("No device detected. Is it plugged in?");
    rs::device * dev = ctx.get_device(0);

    dev->enable_stream(rs::stream::depth, rs::preset::best_quality);
    dev->enable_stream(rs::stream::color, rs::preset::best_quality);
    dev.start();
	std::ostringstream ss; ss << "CPP Point Cloud Example (" << dev.get_name() << ")";
}
void initMap()
{
	GridMap gridMap({"elevation"});
	const double lengthX = resolution * cellnumX;
	const double lengthY = resolution * cellnumY;
	Length length(lengthX, lengthY);
    Position MapPosition = Position(0,0);
	gridMap.setGeometry(length, resolution, MapPosition);
	gridMap.setFrameId("map");
}

void getPointCloud()
{
	const float depth_scale = dev.get_depth_scale();
    const uint16_t * depth_image = (const uint16_t *)dev->get_frame_data(rs::stream::depth);
    const rs::extrinsics extrin = dev.get_extrinsics(rs::stream::depth, tex_stream);
    const rs::intrinsics depth_intrin = dev.get_stream_intrinsics(rs::stream::depth);
    int pointCloudSize = 0;
    //rs::float3 *pcloudPointsRS = reinterpret_cast<const rs::float3 *>(dev.get_frame_data(rs::stream::points));
    while(pcloudPointsRS != NULL) pointCloudSize++；
#if 0
    for(int dy=0; dy<depth_intrin.height; ++dy)
    {
        for(int dx=0; dx<depth_intrin.width; ++dx)
        {
            uint16_t depth_value = depth_image[dy * depth_intrin.width + dx];
            float depth_in_meters = depth_value * scale;
            if(depth_value = 0) continue;

            rs::float2 depth_pixel = {(float)dx, (float)dy};
            rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
            pointCloudSize++;
        }
    }
#endif
}

void PointCloud2GirdMap(rs::float3 *pcloudPointsRS, int pointCloudSize, GridMap &gridMap)
{
    GridMapIterator it(gridMap);
    for(int points_num = 0; points_num < pointCloudSize; points_num++)
    {
      Position position;
      position.x() = pcloudPointsRS.x;
      position.y() = pcloudPointsRS.z;
	  gridMap.getIndex(position, it);
      map.at("elevation", *it) = position.y;
    }
}
