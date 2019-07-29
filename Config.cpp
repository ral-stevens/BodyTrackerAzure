//#include "pch.h"
#include "Config.h"

#include <fstream>
#include <string>
#include <sstream>

// Initialization of static variables.
Config* Config::m_pInstance = nullptr;
int Config::m_countUpdates = 0;

inline static void trim(std::string & str) {
	const std::string chars = "\t\n\v\f\r ";
	str.erase(0, str.find_first_not_of(chars));
	str.erase(str.find_last_not_of(chars) + 1);
}

/** This function is called to create an instance of the class.
    Calling the constructor publicly is not allowed. The constructor
    is private and is only called by this Instance function.
*/
Config* Config::Instance(const std::string & fileName)
{
	if (!m_pInstance) {
		m_pInstance = new Config;
		m_pInstance->load(fileName);
	}
	return m_pInstance;
}


void Config::load(const std::string & fileName)
{
	// Read config from a parameter file
	std::ifstream ifsConfig(fileName);
	if (!ifsConfig.is_open())
	{
		return;
	}
	std::string strLine;
	while (std::getline(ifsConfig, strLine))
	{
		strLine = strLine.substr(0, strLine.find_first_of('#')); // Remove comments
		std::string strKey, strValue;
		std::stringstream ss(strLine);
		if (std::getline(ss, strKey, '='))
			if (std::getline(ss, strValue)) {
				trim(strKey);
				trim(strValue);
				m_mapParams[strKey] = strValue;
			}
				
	}
	ifsConfig.close();
}


bool Config::assign(const std::string & strKey, std::string & strValue)
{
	ConfigParams::iterator it;
	it = m_mapParams.find(strKey);
	if (it == m_mapParams.end())
		return false; // Failed to find the parameter.
	else
	{
		if (strValue.compare(m_mapParams[strKey]) != 0)
		{
			m_countUpdates++;
			strValue = m_mapParams[strKey];
		}		
		return true; // Succeeded in assigning the config parameter
	}
}

bool Config::assign(const std::string & strKey, float & fValue)
{
	ConfigParams::iterator it;
	it = m_mapParams.find(strKey);
	if (it == m_mapParams.end())
		return false; // Failed to find the parameter.
	else
	{
		float fValueNew = stof(m_mapParams[strKey]);
		if (fabs(fValue - fValueNew) > FLOAT_EPSILON)
		{
			m_countUpdates++;
			fValue = fValueNew;
		}
		return true; // Succeeded in assigning the config parameter
	}
}

bool Config::assign(const std::string & strKey, double & fValue)
{
	ConfigParams::iterator it;
	it = m_mapParams.find(strKey);
	if (it == m_mapParams.end())
		return false; // Failed to find the parameter.
	else
	{
		double fValueNew = stof(m_mapParams[strKey]);
		if (fabs(fValue - fValueNew) > FLOAT_EPSILON)
		{
			m_countUpdates++;
			fValue = fValueNew;
		}
		return true; // Succeeded in assigning the config parameter
	}
}

bool Config::assign(const std::string & strKey, bool & bValue)
{
	ConfigParams::iterator it;
	it = m_mapParams.find(strKey);
	if (it == m_mapParams.end())
		return false; // Failed to find the parameter.
	else
	{
		const std::string & strParam = m_mapParams[strKey];
		bool bValueNew = strParam.compare("1") == 0 || strParam.compare("true") == 0 ||
			strParam.compare("True") == 0 || strParam.compare("TRUE") == 0;
		if (bValue != bValueNew)
		{
			m_countUpdates++;
			bValue = bValueNew;
		}
		return true; // Succeeded in assigning the config parameter
	}
}

bool Config::assign(const std::string & strKey, int & iValue)
{
	ConfigParams::iterator it;
	it = m_mapParams.find(strKey);
	if (it == m_mapParams.end())
		return false; // Failed to find the parameter.
	else
	{
		int iValueNew = stoi(m_mapParams[strKey]);
		if (iValue != iValueNew)
		{
			m_countUpdates++;
			iValue = iValueNew;
		}
		return true; // Succeeded in assigning the config parameter
	}
}

void Config::resetCounter()
{
	m_countUpdates = 0;
}

int Config::getUpdateCount()
{
	return m_countUpdates;
}
