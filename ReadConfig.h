#include <map>

// Interpret one config line
class ReadConfigEntry
{
	std::function<void(char *)> pred;
	std::string token;
public:
	ReadConfigEntry() {}
	ReadConfigEntry(std::string token_, std::function<void(char *)> pred_) {
		pred = pred_;
		token = token_;
	}

	void interpretConfigLine(char *achLine)
	{
		char *pszConfig = pszGetConfigEntry(achLine, token.c_str());
		if (pszConfig) {
			pred(pszConfig);
		}
	}

	// does line start with that token? if so, return the buffer of subsequent data
	static char *pszGetConfigEntry(char *achLine, const char *TOKEN)
	{
		char *pszEntry = NULL;
		if (strncmp(achLine, TOKEN, strlen(TOKEN)) == 0) {
			pszEntry = achLine + strlen(TOKEN);
			while (*pszEntry == ' ' || *pszEntry == '=') pszEntry++; // overread blanks and =
		}
		return pszEntry;
	}
};


// Interpret complete config file
class ReadConfig
{
	std::map<std::string, ReadConfigEntry> configReaders;
	boolean wideChar = false;
public:
	ReadConfig(boolean wideChar_) {
		wideChar = wideChar_;
	}

	void addEntry(std::string token, ReadConfigEntry entry) {
		configReaders[token] = entry;
	}

	void addEntry(std::string token, std::function<void(char *)> pred)
	{
		addEntry(token, ReadConfigEntry(token, pred));
	}

	void readFile(FILE *fp) {
		do {
			char achLine[1024];
			achLine[0] = '\0';
			if (wideChar) {
				wchar_t wachLine[1024];
				wachLine[0] = '\0';

				fgetws(wachLine, sizeof(wachLine), fp);
				wcstombs(achLine, wachLine, 1024);
				//printf("wchar Line: %s", achLine);
			}
			else {
				fgets(achLine, sizeof(achLine), fp);
				//printf("char Line: %s", achLine);
			}
			char *pszConfig = NULL;
			for (auto mapEntry : configReaders) {
				mapEntry.second.interpretConfigLine(achLine);
			}


		} while (!feof(fp));
	}

	bool readFile(const char *pszFileName)
	{
		bool bRead = false;
		FILE *fp = fopen(pszFileName, wideChar ? "rb" : "r"); // wchar_t reading as needed for sim.cfg needs binary mode!!!
		if (fp) {
			readFile(fp);
			fclose(fp);
			bRead = true;
		}
		else {
			printf("%s could not be opened!\n", pszFileName);
		}
		return bRead;
	}
};

