#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "TelemetrySenderComponent.generated.h"

class FSocket;
class FInternetAddr;

struct FTelemetryPacket
{
    float DroneX;
    float DroneY;
    float DroneZ;

    float Pitch;
    float Yaw;
    float Roll;

    float TargetX;
    float TargetY;
    float TargetZ;
};

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class ONLINEDRONESIMULATOR_API UTelemetrySenderComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UTelemetrySenderComponent();

protected:
    virtual void BeginPlay() override;

private:
    void SendTelemetry();

    FSocket* Socket = nullptr;
    FTimerHandle TelemetryTimer;
    TSharedPtr<FInternetAddr> RemoteAddress;

    UPROPERTY()
    AActor* TargetActor = nullptr;
};