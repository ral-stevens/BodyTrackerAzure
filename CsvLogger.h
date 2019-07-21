#pragma once
#include <fstream>
#include <mutex>
#include <vector>
#include <utility>

// This is a data logger class in compliance with RAII
class ValueType {
	enum {
		type_uint64 = 0,
		type_f,
		type_str
	} type;

	union {
		const uint64_t * n_;
		const float * f_;
		const std::string * str_;
	};
public:
	ValueType(const uint64_t * n) {
		n_ = n;
		type = type_uint64;
	}
	ValueType(const float * f) {
		f_ = f;
		type = type_f;
	}
	ValueType(const std::string * str) {
		str_ = str;
		type = type_str;
	}

	friend std::ostream & operator<<(std::ostream & lhs, ValueType & rhs);
};

typedef std::vector<std::pair<std::string, ValueType>> vector_header_value_t;

class CsvLogger
{
private:
	std::ofstream m_DataFile;
	CsvLogger::vector_header_value_t m_VectorHeaderValue;
protected:
	std::mutex m_Mutex;
	static std::string s_strDataPath;
	static time_t m_rawtime; // the number of seconds elapsed since 1900 at 00:00 UTC
public:
	// Constructor of BaseLogger
	// Argument "name" is suffixed to the file name.
	CsvLogger(const char * fileName, vector_header_value_t && vectorHeaderValue);
	~CsvLogger();

	template <bool isHeader=false>
	void log()
	{
		if (m_DataFile.is_open() == false) return;
		for (auto headerValue : m_VectorHeaderValue)
		{
			if (isHeader)
				m_DataFile << headerValue.first << ',';
			else
				m_DataFile << headerValue.second << ',';

		}
		m_DataFile << '\n';
	}

protected:

	// Attach timestamp to the beginning of the file name
	void generateFileName(std::string & dest, const char * suffix);

};

