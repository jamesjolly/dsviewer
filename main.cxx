
#include <vector>
#include <exception>
#include <iostream>
using namespace std;

#include <DepthSense.hxx>
using namespace DepthSense;

#include <pcl/visualization/cloud_viewer.h>

const int c_PIXEL_COUNT = 76800; // 320x240
const int c_MIN_Z = 100; // discard points closer than this
const int c_MAX_Z = 2000; // discard points farther than this

Context g_context;
DepthNode g_dnode;

uint32_t g_dFrames = 0;
bool g_bDeviceFound = false;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");

// event handler: new depth frame
void onNewDepthSample(DepthNode node, DepthNode::NewSampleReceivedData data)
{
    cout << "frame " << g_dFrames << "\n";
    for (int i = 0; i < c_PIXEL_COUNT; i++)
    {
    	if (data.vertices[i].z > c_MAX_Z || data.vertices[i].z < c_MIN_Z)
    	{
    	    cloud->points[i] = pcl::PointXYZ(0, 0, 0);
    	    continue;
    	}
        cloud->points[i].x = data.vertices[i].x;
        cloud->points[i].y = data.vertices[i].y;
        cloud->points[i].z = data.vertices[i].z;
    }
    viewer.showCloud(cloud);
    g_dFrames++;
}

// we're only using the vertices feed
void configureDepthNode(Node node)
{
    if (node.is<DepthNode>() && !g_dnode.isSet())
    {
        g_dnode = node.as<DepthNode>();
        g_dnode.newSampleReceivedEvent().connect(&onNewDepthSample);
        DepthNode::Configuration config = g_dnode.getConfiguration();
        config.frameFormat = FRAME_FORMAT_QVGA;
        config.framerate = 25;
        config.mode = DepthNode::CAMERA_MODE_CLOSE_MODE;
        config.saturation = true;
        g_dnode.setEnableVertices(true);
        try
        {
            g_context.requestControl(g_dnode, 0);
            g_dnode.setConfiguration(config);
            cout << "depth node connected\n";
        }
        catch (Exception& e)
        {
            cout << "Exception: " << e.what() << "\n";
        }
        g_context.registerNode(node);
    }
}

// event handler: setup vertices feed
void onNodeConnected(Device device, Device::NodeAddedData data)
{
    configureDepthNode(data.node);
}

// event handler: tear down vertices feed
void onNodeDisconnected(Device device, Device::NodeRemovedData data)
{
    if (data.node.is<DepthNode>() && (data.node.as<DepthNode>() == g_dnode))
    {
        g_dnode.unset();
        cout << "depth node disconnected\n";
    }
}

// event handler: device plugged in
void onDeviceConnected(Context context, Context::DeviceAddedData data)
{
    if (!g_bDeviceFound)
    {
        data.device.nodeAddedEvent().connect(&onNodeConnected);
        data.device.nodeRemovedEvent().connect(&onNodeDisconnected);
        g_bDeviceFound = true;
    }
}

// event handler: device unplugged
void onDeviceDisconnected(Context context, Context::DeviceRemovedData data)
{
    g_bDeviceFound = false;
    cout << "device disconnected\n";
}

int main(int argc, char** argv)
{
    g_context = Context::create("localhost");
    g_context.deviceAddedEvent().connect(&onDeviceConnected);
    g_context.deviceRemovedEvent().connect(&onDeviceDisconnected);
    cloud->points.resize(c_PIXEL_COUNT);

    // get list of devices already connected
    vector<Device> da = g_context.getDevices();

    // only use first device
    if (da.size() >= 1)
    {
        g_bDeviceFound = true;
        da[0].nodeAddedEvent().connect(&onNodeConnected);
        da[0].nodeRemovedEvent().connect(&onNodeDisconnected);
        vector<Node> na = da[0].getNodes();
        cout << "found " << (int)na.size() << " nodes\n";
        for (int n = 0; n < (int)na.size(); n++)
        {
            configureDepthNode(na[n]);
        }
    }

    g_context.startNodes();
    g_context.run();
    g_context.stopNodes();
    if (g_dnode.isSet())
    {
    	g_context.unregisterNode(g_dnode);
    }
    return 0;
}

