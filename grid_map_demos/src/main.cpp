
void main()
{
    rs2GridMap rsMap;
    rsMap.initDevice();
    rsMap.initMap();
    while(1)
    {
       rsMap.dev->wait_for_frames();
       rsMap.getPointCloud();
       rsMap.PointCloud2GirdMap();
    }

}
