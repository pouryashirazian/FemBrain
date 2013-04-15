#include "AssetManager.h"
#include "PS_Base/PS_Logger.h"

Asset::Asset():m_isSerializeable(false), m_tsAccessed(0) {
}

Asset::~Asset() {

}

bool Asset::read(const char *chrFilePath) {
    return false;
}

bool Asset::write(const char *chrFilePath) {
    return false;
}


AssetManager::AssetManager() {

}

