using System;
using System.Runtime.InteropServices;
using System.Threading;
using System.Threading.Tasks;
using System.Diagnostics;

public class KetiROSBridgeWrapper : IDisposable{
  private IntPtr _nativeObject;

  [DllImport("libclient.so")]
  private static extern void ros_init(string node_name);

  [DllImport("libclient.so")]
  private static extern IntPtr KetiROSBridge_Create(string host_name, string cmd_host_name);

  [DllImport("libclient.so")]
  private static extern void KetiROSBridge_Destroy(IntPtr obj);

  [DllImport("libclient.so")]
  private static extern void KetiROSBridge_StartBridge(IntPtr obj);

  [DllImport("libclient.so")]
  private static extern void KetiROSBridge_ClientStartListen(IntPtr obj);

  [DllImport("libclient.so")]
  private static extern void KetiROSBridge_ServerStartStream(IntPtr obj);

  public KetiROSBridgeWrapper(string node_name, string host_name, string cmd_host_name){
    ros_init(node_name);
    _nativeObject = KetiROSBridge_Create(host_name, cmd_host_name);
  }

  public void StartBridge(){
    try{
      KetiROSBridge_StartBridge(_nativeObject);
    }
    catch(Exception ex){
      Console.WriteLine($"An error occuredd: {ex.Message}");
    }
    finally{
      Console.ReadKey();
    }
  }
  public void ClientStartListen(){
    try{
      KetiROSBridge_ClientStartListen(_nativeObject);
    }
    catch(Exception ex){
      Console.WriteLine($"An error occuredd: {ex.Message}");
    }
    finally{
      Console.ReadKey();
    }
  }
  public void ServerStartStream(){
    try{
      KetiROSBridge_ServerStartStream(_nativeObject);
    }
    catch(Exception ex){
      Console.WriteLine($"An error occuredd: {ex.Message}");
    }
    finally{
      Console.ReadKey();
    }
  }
  public void Dispose(){
    if( _nativeObject != IntPtr.Zero){
      KetiROSBridge_Destroy(_nativeObject);
      _nativeObject = IntPtr.Zero;
    }
  }
  
  ~KetiROSBridgeWrapper(){
    Dispose();
  }
}

class OSIClientCsharp{

  private static string node_name="grpc_client";
  private static string host_name="localhost:50051";
  private static string cmd_host_name="localhost:50052";
  
  public static async Task Main(string[] args){

    Console.WriteLine(" In Main : ");

    using (var keti_ros_bridge = new KetiROSBridgeWrapper(node_name, host_name, cmd_host_name))
    using (var cts = new CancellationTokenSource())
    {
      try{
        var bridgeTask = Task.Run(()=> RunWithCancel(keti_ros_bridge.StartBridge, cts.Token));
        var clientListenTask = Task.Run(()=> RunWithCancel(keti_ros_bridge.ClientStartListen, cts.Token));
        var serverStreamTask = Task.Run(()=> RunWithCancel(keti_ros_bridge.ServerStartStream, cts.Token));

        Console.WriteLine("All tasks started.");
        Console.ReadKey();

        Console.CancelKeyPress += (sender, e) =>
        {
          e.Cancel = true;
          cts.Cancel();
        };

        await Task.WhenAll(clientListenTask, serverStreamTask, bridgeTask);
      }
      catch(Exception ex){
        Console.WriteLine($"An error occuredd: {ex.Message}");
      }
      finally{
        Console.WriteLine("Program ended. Press Enter to exit.");
        Console.ReadKey();
      }
    }
  }

  static void RunWithCancel(Action action, CancellationToken ct){
    while( !ct.IsCancellationRequested){
      action();
      Thread.Sleep(100);
    }
  }
}