#include "stdafx.h"
#include "CsvLogger.h"
#include <map>
#include <sstream>
#include "Config.h"

std::ostream & operator<<(std::ostream & lhs, ValueType & rhs) {
	switch (rhs.type) {
	case ValueType::type_uint64:
		return lhs << *(rhs.n_);
	case ValueType::type_f:
		return lhs << *(rhs.f_);
	case ValueType::type_str:
		return lhs << *(rhs.str_);
	default:
		throw(std::runtime_error("CsvLogger: Unknow value type."));
	}
}

std::string CsvLogger::s_strDataPath = "";
time_t CsvLogger::m_rawtime = 0;
CsvLogger::CsvLogger(const char * name, vector_header_value_t && vectorHeaderValue) :
	m_VectorHeaderValue(std::move(vectorHeaderValue))
{
	bool enabled = false;
	Config::Instance()->assign("CsvLogger/enabled", enabled);
	if (enabled)
	{
		Config::Instance()->assign("CsvLogger/dataPath", s_strDataPath);
		if (m_rawtime == 0)
			time(&m_rawtime); // obtain current time
		// Open a csv file
		std::string fileName;
		generateFileName(fileName, name);
		m_DataFile.open(fileName, std::ofstream::out); // Open the csv file
		if (m_DataFile.is_open() == false)
			throw std::runtime_error("Cannot open file\n" + fileName +
				"\n\nBaseLogger::openDataFile(const char * name)");
		else
			log<true>();
	}
}


CsvLogger::~CsvLogger()
{
	if (m_DataFile.is_open()) {
		m_DataFile.close();
	}
}

void CsvLogger::generateFileName(std::string & dest, const char * suffix)
{
	//system("mkdir data");
	std::string strSuffix(suffix);
	static std::map<std::string, int> MapCounter;
	if (MapCounter.find(strSuffix) == MapCounter.end())
		MapCounter[strSuffix] = 0;
	else
		MapCounter[strSuffix]++;

	struct tm *timeinfo = localtime(&m_rawtime); // represent current time using struct
	std::stringstream ssFileName; // Construct the name of the csv file
	ssFileName << "data_"
		<< timeinfo->tm_year + 1900 << std::setfill('0')
		<< std::setw(2) << timeinfo->tm_mon + 1
		<< std::setw(2) << timeinfo->tm_mday << "_"
		<< std::setw(2) << timeinfo->tm_hour << "_"
		<< std::setw(2) << timeinfo->tm_min << "_"
		<< std::setw(2) << timeinfo->tm_sec << "_"
		<< suffix;
	if (strSuffix.find('.') == std::string::npos)
		ssFileName << MapCounter[strSuffix] << ".csv";
	if (!s_strDataPath.empty() && s_strDataPath.back() != '\\')
		s_strDataPath += '\\';
	dest = s_strDataPath + ssFileName.str();
}


