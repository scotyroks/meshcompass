#include "CompassModule.h"
#include "mesh/MeshTypes.h"
#include "mesh/NodeDB.h"
#include "wiring.h"
#include "PositionModule.h"

CompassModule *compassModule;

CompassModule::CompassModule()
{
    compassModule = this;

    // Initialize configuration settings
    sdaPin = COMPASS_SDA_PIN;
    sclPin = COMPASS_SCL_PIN;
    ledPin = COMPASS_LED_PIN;
    ledCount = COMPASS_LED_COUNT;

    initCompass();
    initLedRing();
}

int32_t CompassModule::runOnce()
{
    compass.read();
    LOG_DEBUG("Compass heading: %d", compass.getAzimuth());
    updateLedRing();
    return my_interval; // Return the interval for the next run
}

void CompassModule::initCompass()
{
    Wire.setPins(sclPin, sdaPin);
    Wire.begin();
    compass.init();
    LOG_INFO("Compass initialized");
}

void CompassModule::initLedRing()
{
    ledRing = new Adafruit_NeoPixel(ledCount, ledPin, NEO_GRB + NEO_KHZ800);
    ledRing->begin();
    ledRing->show(); // Initialize all pixels to 'off'
    LOG_INFO("LED ring initialized");
}

void CompassModule::updateLedRing()
{
    ledRing->clear();

    if (!positionModule->getLatitude() || !positionModule->getLongitude())
        return; // No local position yet

    float localLat = (float)positionModule->getLatitude() / 1e7;
    float localLon = (float)positionModule->getLongitude() / 1e7;
    float heading = compass.getAzimuth();

    for (size_t i = 0; i < nodeDB->getNumMeshNodes(); i++)
    {
        meshtastic_NodeInfoLite *nodeInfo = nodeDB->getMeshNodeByIndex(i);
        if (nodeInfo->position.latitude_i && nodeInfo->position.longitude_i)
        {
            float neighborLat = (float)nodeInfo->position.latitude_i / 1e7;
            float neighborLon = (float)nodeInfo->position.longitude_i / 1e7;

            float bearing = calculateBearing(localLat, localLon, neighborLat, neighborLon);
            float relativeBearing = fmod((bearing - heading + 360), 360);

            LOG_DEBUG("Node %d: bearing=%f, relativeBearing=%f", nodeInfo->num, bearing, relativeBearing);

            int ledIndex = (int)(relativeBearing / (360.0 / ledCount));
            ledRing->setPixelColor(ledIndex, ledRing->Color(255, 0, 0)); // Red for now
        }
    }
    ledRing->show();
}

#include <math.h>

float CompassModule::calculateBearing(float lat1, float lon1, float lat2, float lon2)
{
    float dLon = lon2 - lon1;
    float y = sin(dLon) * cos(lat2);
    float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    float bearing = atan2(y, x);
    return fmod((bearing * 180 / M_PI + 360), 360);
}
