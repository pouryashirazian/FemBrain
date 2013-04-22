#include "AssetManager.h"
#include "PS_Base/PS_Logger.h"

Asset::Asset():m_isSerializeable(false), m_tsAccessed(0) {
}

Asset::~Asset() {

}

bool Asset::load(const char *chrFilePath) {
    return false;
}

bool Asset::store(const char *chrFilePath) {
    return false;
}


AssetManager::AssetManager() {

}

