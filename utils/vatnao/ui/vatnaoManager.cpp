#include "vatnaoManager.hpp"

#include "../app/appStatus.hpp"
#include "../app/vatnaoFrameInfo.hpp"
#include "frameTools/readImage.hpp"
#include "frameTools/readBoundingBoxes.hpp"

VatnaoManager::VatnaoManager(AppAdaptor *a){
    vatnao_query_.region_index = 0;
    vatnao_query_.subregion_index = 0;
    appAdaptor = a;
    AppStatus status = appAdaptor->getStatus();
    topImage = new QImage(status.numTopCameraCols, status.numTopCameraRows, QImage::Format_RGB32);
    botImage = new QImage(status.numBotCameraCols, status.numBotCameraRows, QImage::Format_RGB32);
}

VatnaoManager::~VatnaoManager(){}

AppStatus VatnaoManager::getAppStatus() {
    return appAdaptor->getStatus();
}

void VatnaoManager::nextFrame(){
    appAdaptor->forward();
    refreshImages();
}

void VatnaoManager::nextTenFrames(){
    appAdaptor->forward(10);
    refreshImages();
}

void VatnaoManager::prevFrame(){
    appAdaptor->back();
    refreshImages();
}

void VatnaoManager::prevTenFrames(){
    appAdaptor->back(10);
    refreshImages();
}

void VatnaoManager::nextTenRegions() {
    updateRegionIndex(10);
    reprocess();
}

void VatnaoManager::nextRegion() {
    updateRegionIndex(1);
    reprocess();
}

void VatnaoManager::prevRegion() {
    updateRegionIndex(-1);
    reprocess();
}

void VatnaoManager::prevTenRegions() {
    updateRegionIndex(-10);
    reprocess();
}

void VatnaoManager::nextTenSubRegions() {
    updateSubregionIndex(10);
    reprocess();
}

void VatnaoManager::nextSubRegion() {
    updateSubregionIndex(1);
    reprocess();
}

void VatnaoManager::prevSubRegion() {
    updateSubregionIndex(-1);
    reprocess();
}

void VatnaoManager::prevTenSubRegions() {
    updateSubregionIndex(-10);
    reprocess();
}

void VatnaoManager::updateBooleanOption(string option_name, bool value) {
    vatnao_query_ = appAdaptor->getVatnaoQuery();
    vatnao_query_ = vatnao_query_.withUpdatedOption(option_name, VatnaoOption::string_from_boolean(value));
    appAdaptor->setVatnaoQuery(vatnao_query_);
}

void VatnaoManager::updateStringOption(string option_name, string option) {
    vatnao_query_ = appAdaptor->getVatnaoQuery().withUpdatedOption(option_name, option);
    appAdaptor->setVatnaoQuery(vatnao_query_);
}

void VatnaoManager::updateNumericOption(string option_name, double value) {
    vatnao_query_ = appAdaptor->getVatnaoQuery().withUpdatedNumericOption(option_name, value);
    appAdaptor->setVatnaoQuery(vatnao_query_);
}

void VatnaoManager::updateRegionIndex(int change) {
    vatnao_query_ = appAdaptor->getVatnaoQuery().withUpdatedRegionIndex(change);
    appAdaptor->setVatnaoQuery(vatnao_query_);
}

void VatnaoManager::updateSubregionIndex(int change) {
    vatnao_query_ = appAdaptor->getVatnaoQuery().withUpdatedSubregionIndex(change);
    appAdaptor->setVatnaoQuery(vatnao_query_);
}

void VatnaoManager::reprocess() {
    appAdaptor->process();
    refreshImages();
}

void VatnaoManager::refreshImages(){
    VatnaoFrameInfo frameInfo = appAdaptor->getFrameInfo();
    AppStatus status = appAdaptor->getStatus();

    frameToQImage(frameInfo, true, topImage);
    frameToQImage(frameInfo, false, botImage);

    drawRegionBoundingBoxes(frameInfo, topImage, botImage);
    drawBallBoundingBoxes(frameInfo, topImage, botImage);
    drawFieldBoundaries(frameInfo, topImage, botImage);
    drawFieldLines(frameInfo, topImage, botImage);
    drawFieldPoints(frameInfo, topImage, botImage);
    drawGoalPosts(frameInfo, topImage, botImage);
    drawRobots(frameInfo, topImage, botImage);

    drawHighlightedBoundingBox(frameInfo, topImage, botImage, status.region_index);

    topImageChanged(QPixmap::fromImage(*topImage));
    botImageChanged(QPixmap::fromImage(*botImage));

    debugInfoChanged(status, frameInfo.previewRegion, frameInfo.annotatedRegion);

    frameInfo.print();
}

