#include "TelemetrySenderComponent.h"
#include "Sockets.h"
#include "SocketSubsystem.h"
#include "IPAddress.h"
#include "Common/UdpSocketBuilder.h"
#include "EngineUtils.h"

UTelemetrySenderComponent::UTelemetrySenderComponent()
{
    PrimaryComponentTick.bCanEverTick = false;
    SetIsReplicated(false);
}

void UTelemetrySenderComponent::BeginPlay()
{
    Super::BeginPlay();

	// create UDP socket
    Socket = FUdpSocketBuilder(TEXT("TelemetrySocket"))
        .AsReusable().WithBroadcast();

    RemoteAddress =
        ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)
        ->CreateInternetAddr();

    bool bIsValid = false;

    RemoteAddress->SetIp(TEXT("127.0.0.1"), bIsValid);
    RemoteAddress->SetPort(9000);

    if (Socket)
    {
        UE_LOG(LogTemp, Warning, TEXT("UDP socket created."));
    }

	// search for the target actor in the world
    for (AActor* Actor : TActorRange<AActor>(GetWorld()))
    {
        if (Actor->ActorHasTag("Target"))
        {
            TargetActor = Actor;
            break;
        }
    }

    if (TargetActor)
    {
        UE_LOG(LogTemp, Warning, TEXT("Found target: %s"), *TargetActor->GetName());
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("Could not find target actor!"));
    }

	// start sending telemetry data at 50 Hz
    GetWorld()->GetTimerManager().SetTimer(
        TelemetryTimer,
        this,
        &UTelemetrySenderComponent::SendTelemetry,
        0.02f,
        true
    );

    UE_LOG(LogTemp, Warning, TEXT("Telemetry timer started."));

    bool TimerActive =
        GetWorld()->GetTimerManager().IsTimerActive(TelemetryTimer);

    UE_LOG(
        LogTemp,
        Warning,
        TEXT("Timer active: %s"),
        TimerActive ? TEXT("YES") : TEXT("NO")
    );
}

void UTelemetrySenderComponent::SendTelemetry()
{
    UE_LOG(LogTemp, Warning, TEXT("SendTelemetry called"));

    if (!Socket)
    {
        UE_LOG(LogTemp, Warning, TEXT("No socket"));
        return;
    }

    if (!TargetActor)
    {
        UE_LOG(LogTemp, Warning, TEXT("No target actor"));
        return;
	}

    AActor* Drone = GetOwner();

    if (!Drone)
    {
        UE_LOG(LogTemp, Warning, TEXT("No drone"));
        return;
    }


    FVector DroneLocation = Drone->GetActorLocation();
    FRotator DroneRotation = Drone->GetActorRotation();

    FVector TargetLocation = TargetActor->GetActorLocation();


    FTelemetryPacket Packet;

    Packet.DroneX = DroneLocation.X;
    Packet.DroneY = DroneLocation.Y;
    Packet.DroneZ = DroneLocation.Z;

    Packet.Pitch = DroneRotation.Pitch;
    Packet.Yaw = DroneRotation.Yaw;
    Packet.Roll = DroneRotation.Roll;

    Packet.TargetX = TargetLocation.X;
    Packet.TargetY = TargetLocation.Y;
    Packet.TargetZ = TargetLocation.Z;


    int32 BytesSent = 0;

    Socket->SendTo(
        reinterpret_cast<uint8*>(&Packet),
        sizeof(Packet),
        BytesSent,
        *RemoteAddress
    );

    UE_LOG(
        LogTemp,
        Warning,
        TEXT("Sent telemetry: Drone %.1f %.1f %.1f"),
        Packet.DroneX,
        Packet.DroneY,
        Packet.DroneZ
    );
}