/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef _GRASP_PLUGIN_MANAGER_H_
#define _GRASP_PLUGIN_MANAGER_H_

#include <string>
#include <boost/regex.hpp>
#include <QLibrary>	/* modified by qtconv.rb 1st rule*/  

namespace grasp{
	
	typedef void* (*GrasplotEntry)(cnoid::BodyPtr body, cnoid::Link *base, cnoid::Link *palm);

    class GraspPluginManager
    {
    public:

        GraspPluginManager();
        ~GraspPluginManager();

        void scanPluginFiles(const std::string& pathString);
		void* loadGrasplotPlugin(cnoid::BodyPtr body, cnoid::Link *base, cnoid::Link *palm, std::string className);
	private:
        enum PluginStatus { NOT_LOADED, LOADED, ACTIVE, INVALID, CONFLICT };

        std::ostream& os;

        boost::regex pluginNamePattern;
        boost::regex hiddenFileDirPattern;

        struct PluginInfo {
            QLibrary dll;	/* modified by qtconv.rb 2nd rule*/  
            std::string pathString;
            void* plugin;
            std::string name;
            int status;
        };
        typedef boost::shared_ptr<PluginInfo> PluginInfoPtr;

        typedef std::vector<PluginInfoPtr> PluginInfoArray;
        PluginInfoArray pluginInfoList;

        typedef std::map<std::string, PluginInfoPtr> PluginMap;
        PluginMap pathToPluginInfoMap;

    };
}

#endif
