#include <filesystem>
#include <functional>
using namespace std::experimental::filesystem::v1;

class HoistP3DBase
{
public:
	static bool find_file_recursive(const path & dir_path, std::string file_name, std::function<void(std::string)> func)
	{
		directory_iterator end_itr; // default construction yields past-the-end
		for (directory_iterator itr(dir_path); itr != end_itr; ++itr)
		{
			if (is_directory(itr->status()))
			{
				find_file_recursive(itr->path(), file_name, func);
			}
			else if (itr->path().filename() == file_name)
			{
				// apply routine to sim.cfg:
				func.operator()(itr->path().string());
			}
		}
		return true;
	}
};

