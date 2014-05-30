using UnityEngine;
using System.Collections;

public class NetworkControl : MonoBehaviour 
{
	public enum States
	{
		Disabled,
		Initializing,
		Client,
		ClientError,
		Server,
	}
	
	public States State = States.Disabled;
	
	public string Address = "localhost";
	
	public int Port = 44044;
	
	public int MaxPlayers = 16;
	
	private NetworkConnectionError clientError;
	
	public void OnGUI()
	{
		GUILayout.BeginVertical();
		GUILayout.BeginHorizontal();
		GUILayout.Label( "Address: " );
		Address = GUILayout.TextField( Address );
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal();
		GUILayout.Label( "Port: " );
		int.TryParse( GUILayout.TextField( Port.ToString() ), out Port );
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal();
		GUILayout.Label( "Max Players: " );
		int.TryParse( GUILayout.TextField( MaxPlayers.ToString() ), out MaxPlayers );
		GUILayout.EndHorizontal();
		switch ( State )
		{
		case States.Disabled:
			if ( GUILayout.Button( "Start Server" ) )
			{
				State = States.Initializing;
				Network.InitializeServer( MaxPlayers, Port, !Network.HavePublicAddress() );
			}
			if ( GUILayout.Button( "Connect Client" ) )
			{
				State = States.Initializing;
				Network.Connect( Address, Port );
			}
			break;
		case States.ClientError:
			GUILayout.Label( "failed to connect client: " + clientError );
			if ( GUILayout.Button( "Connect Client" ) )
			{
				Network.Connect( Address, Port );
				State = States.Initializing;
			}
			break;
		case States.Client:
			if ( !Network.isClient || GUILayout.Button( "Disconnect Client" ) )
			{
				Network.Disconnect();
				State = States.Disabled;
			}
			break;
		case States.Server:
			foreach ( NetworkPlayer player in Network.connections )
			{
				GUILayout.Label( player.ipAddress + ":" + player.port );
			}
			if ( GUILayout.Button( "Stop Server" ) )
			{
				Network.Disconnect();
				State = States.Disabled;
			}
			break;
		case States.Initializing:
			GUILayout.Label( "initializing..." );
			if ( GUILayout.Button( "Cancel" ) )
			{
				Network.Disconnect();
				State = States.Disabled;
			}
			break;
		}
		GUILayout.EndVertical();
	}
	
	public void OnServerInitialized()
	{
		Debug.Log( "server initialized" );
		State = States.Server;
	}
	
	public void OnPlayerConnected( NetworkPlayer player )
	{
		Debug.Log( "player connected " + player );
	}
	
	public void OnConnectedToServer()
	{
		Debug.Log( "connected to server" );
		State = States.Client;
	}
	
	public void OnFailedToConnect( NetworkConnectionError error )
	{
		Debug.Log( "failed to connect to server " + Address + ":" + Port + ": " + error );
		State = States.ClientError;
		clientError = error;
	}
}
