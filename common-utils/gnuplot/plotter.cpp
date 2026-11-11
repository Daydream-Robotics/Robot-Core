#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <map>

//struct to hold files belonging to a basename/run
struct LogSet {
    //run identifier
    std::string base_name;
    //planned trajectory
    std::string target_pos_file;
    //actual trajectory
    std::string actual_pos_file;
    //tracking error data
    std::string error_pos_file;
    //vector to hold multiple value data files
    std::vector<std::string> value_files;
};

//struct to hold start pose to offset the field map
struct StartPose {
    double x = 0;
    double y = 0;
};

//recenter trajectory files so plots align to real starting pose
std::string generate_recentered_file(const std::string& original_file, const std::string& outDir, const std::string& suffix, double offsetX, double offsetY, bool isTarget) {
    //check for file existence
    if (original_file.empty() || !std::filesystem::exists(original_file)) {
        return "";
    }

    //temporary output file for modified trajectory
    std::string recentered_name = outDir + "/temp_" + suffix + ".dat";
    //output file
    std::ofstream out(recentered_name);
    //input file
    std::ifstream in(original_file);

    std::string line;

    //process each logged line
    while (std::getline(in, line)) {
        //skip comments or empty lines
        if (line.empty() || line[0] == '#') {
            continue;
        }

        double time, x, y;

        //only process trajectory formatted files
        if (isTarget) {
            if (sscanf(line.c_str(), "%lf %lf %lf", &time, &x, &y) == 3) {
                //apply coordinate offset
                out << time << " " << (x + offsetX) << " " << (y + offsetY) << "\n";
            }
        }
    }

    //ensure flushing
    out.close();
    return recentered_name;
}

//escape file paths for shell usage
std::string escapePath(const std::string& path) {
    std::string result;
    //escape single quotes for shell compatibility
    for (char c : path) {
        if (c == '\'') result += "\\'";
        else result += c;
    }
    return result;
}

//scan directories and group logs into runs
std::vector<LogSet> findLogs(const std::string& dir) {
    std::vector<LogSet> sets;
    std::map<std::string, LogSet> groups;

    //iterate over SD card dir
    for (const auto& entry : std::filesystem::directory_iterator(dir)){
        std::string filename = entry.path().filename().string();
        //filter for only .dat files
        size_t dot_pos = filename.find_last_of('.');
        if (dot_pos == std::string::npos || filename.substr(dot_pos) != ".dat") {
            continue;
        }

        //split base name and suffix type
        size_t underscore_pos = filename.find_last_of('_');
        if (underscore_pos == std::string::npos) {
            continue;
        }
        std::string basename = filename.substr(0, underscore_pos);
        std::string type = filename.substr(underscore_pos + 1, dot_pos - underscore_pos -1);

        //classify file into correct category
        if (type == "target") {
            groups[basename].target_pos_file = entry.path().string();
        }
        else if (type == "actual") {
            groups[basename].actual_pos_file = entry.path().string();
        }
        else if (type == "error") {
            groups[basename].error_pos_file = entry.path().string();
        }
        else {
            //treat anything else as a value log
            groups[basename].value_files.push_back(entry.path().string());
        }
        groups[basename].base_name = basename;
    }

    //flatten the map into a vector
    for (auto& [base, set] : groups) {
        sets.push_back(set);
    }

    return sets;
}

//generate gnuplot script for trajectory visualiuzation
void generateTrajectoryGNUPlot (const LogSet& log, const std::string& outDir, const StartPose& start) {
    //recenter trajectory
    std::string temp_target = generate_recentered_file(log.target_pos_file, outDir, "target", start.x, start.y, true);
    std::string temp_actual = generate_recentered_file(log.actual_pos_file, outDir, "actual", start.x, start.y, true);

    //make gnuplot script file
    std::string gp_file = outDir + "/plot_" + log.base_name + ".gp";

    std::ofstream gp(gp_file);

    //setup plot styling
    gp << "#!/usr/bin/env gnuplot\n";
    gp << "set terminal pngcairo size 1600,800 enhanced font 'Arial,12' background rgb '#1e1e1e'\n";
    gp << "set output '" << outDir << "/" << log.base_name << "_dashboard.png'\n";
    
    //dark theme styling
    gp << "set border lc rgb '#e6e6e6'\n";
    gp << "set tics tc rgb '#e6e6e6'\n";
    gp << "set key tc rgb '#e6e6e6'\n";
    gp << "set grid lc rgb '#3a3a3a'\n";

    //two panel layout (path+error)
    gp << "set multiplot layout 1,2 title '" << log.base_name << "' textcolor rgb '#e6e6e6'\n\n";

    //left plot: trajectory
    gp << "set size ratio 1\n";

    gp << "set xrange [-72:72]\n";
    gp << "set yrange [-72:72]\n";

    gp << "set xtics 24\n";
    gp << "set ytics 24\n";

    gp << "set mxtics 2\n";
    gp << "set mytics 2\n";

    //label trajectory plot
    gp << "set title 'Path' textcolor rgb '#e6e6e6'\n";
    gp << "set xlabel 'x (inches)' textcolor rgb '#e6e6e6'\n";
    gp << "set ylabel 'y (inches)' textcolor rgb '#e6e6e6'\n";
    
    gp << "set grid xtics ytics mxtics mytics\n";
    gp << "set key top left\n";

    //draw x and y axes through the origin
    gp << "set zeroaxis lc rgb '#666666' lw 1\n";

    //outline the boundry
    gp << "set object 1 rectangle from -72,-72 to 72,72 fillstyle empty border lc rgb '#e6e6e6' lw 2\n";

    //start constructing the trajectory plot
    gp << "plot ";

    bool first = true;
    //helper to add dataset to plot
    auto add = [&](const std::string& file_path, const std::string& using_cols, const std::string& style, const std::string& title, const std::string& color) {
        //skip missing or empty log files
        if (file_path.empty() || !std::filesystem::exists(file_path)) {
            return;
        }

        //seperate multiple datasets with commas
        if (!first) {
            gp << ", \\\n     ";
        }

        first = false;

        //add a formatted gnuplot dataset entry
        gp << "'" << escapePath(file_path) << "' using " << using_cols << " " << style << " lc rgb '" << color << "' title '" << title << "'";
    };

    //plot the target and actual trajectories
    add(temp_target, "2:3", "with lines lw 2", "target", "#4a9eff");
    add(temp_actual, "2:3", "with lines lw 1", "actual", "#5cdb5c");

    //seperate first and second plot
    gp << "\n\n";

    //right plot: error over time
    gp << "set size ratio 0\n";
    gp << "set autoscale x\n";

    gp << "set yrange [0:6]\n";

    gp << "set ytics 1\n";
    gp << "set mytics 2\n";

    //remove field outline from second plot
    gp << "unset object 1\n";

    //configure title and labels
    gp << "set title 'Position Error' textcolor rgb '#e6e6e6'\n";
    gp << "set xlabel 'time (s)' textcolor rgb '#e6e6e6'\n";
    gp << "set ylabel 'error (inches)' textcolor rgb '#e6e6e6'\n";
    
    //enable bg grid
    gp << "set grid\n";

    gp << "set key top left\n";

    //plot error as filled region beneath the curve
    gp << "plot '" << escapePath(log.error_pos_file) << "' using 1:2 with filledcurves y1=0 lc rgb '#5a2222' title '', \\\n";
    //overlay error lien on top of shaded area
    gp << "     '" << escapePath(log.error_pos_file) << "' using 1:2 with lines lw 2 lc rgb '#ff6b6b' title 'error'\n";

    //finish the two panel layout
    gp << "unset multiplot\n";

    //write the gnuplot script to disk
    gp.close();

    //execute the gnuplot script
    std::system(("gnuplot '" + escapePath(gp_file) + "'").c_str());
    //notify user where the dashboar image was written
    std::cout << "Generated: " << outDir << "/" << log.base_name << "_dashboard.png\n";
    
    //delete temp recentered trajectory files
    if (!temp_target.empty()) {
        std::filesystem::remove(temp_target);
    }
    if (!temp_actual.empty()) {
        std::filesystem::remove(temp_actual);
    }
}

//generate every plot for value files
void generateAllValueFileGNUPlots(const LogSet& log, const std::string& outDir) {
    //skip plotting if no value logs exist
    if (log.value_files.empty()) {
        return;
    }

    //path to generated gnuplot script
    std::string gp_file = outDir + "/plot_" + log.base_name + "_values.gp";
    //create gnuplot script
    std::ofstream gp(gp_file);

    //configure output image
    gp << "#!/usr/bin/env gnuplot\n";
    gp << "set terminal pngcairo size 1600,800 enhanced font 'Arial,12' background rgb '#1e1e1e'\n";
    gp << "set output '" << outDir << "/" << log.base_name << "_values.png'\n";
    
    //configure plot appearance
    gp << "set border lc rgb '#e6e6e6'\n";
    gp << "set tics tc rgb '#e6e6e6'\n";
    gp << "set key tc rgb '#e6e6e6'\n";
    gp << "set grid lc rgb '#3a3a3a'\n";

    //configure title and axis labels
    gp << "set title '" << log.base_name << " - Value Plots' textcolor rgb '#e6e6e6'\n";
    gp << "set xlabel 'time (s)' textcolor rgb '#e6e6e6'\n";
    gp << "set ylabel 'value' textcolor rgb '#e6e6e6'\n";
    
    //display bg grid and place legend in top left
    gp << "set grid\n";
    gp << "set key top left\n";

     //start constructing the plots
    gp << "plot ";

    //track whether another dataset has already been added
    bool first = true;

    //add each value file to the combined plot
    for (const std::string& value_file : log.value_files) {
        //get the filename w/o the extension
        std::string stem = std::filesystem::path(value_file).stem().string();
        //extract basename to use as plot title
        size_t underscore_pos = stem.find_last_of('_');
        std::string label = (underscore_pos != std::string::npos) ? stem.substr(underscore_pos + 1) : stem;

        //seperate multiple datasets w/ commas
        if(!first) {
            gp << ", \\\n     ";
        }

        first = false;

        //plot value v time
        gp << "'" << escapePath(value_file) << "' every ::1 using 1:2 with lines lw 2 title '" << label << "'";
    }

    //finish plot cmd
    gp << "\n";

    //write gnuplot to disk
    gp.close();
    //execute gnuplot script
    std::system(("gnuplot '" + escapePath(gp_file) + "'").c_str());
    //notify user where the plot was written
    std::cout << "Generated: " << outDir << "/" << log.base_name << "_values.png\n";
}

//generate a single plot of a value over time
void generateValueFileGNUPlot(const std::string& path, const std::string& outDir) {
    //extract the filenmae w/o extension
    std::string filename = std::filesystem::path(path).stem().string();
    //extract basename to use as plot title
    size_t underscore_pos = filename.find_last_of('_');
    std::string label = (underscore_pos != std::string::npos) ? filename.substr(underscore_pos + 1) : "value";

    //path to generated gnuplot script
    std::string gp_file = outDir + "/plot_" + filename + ".gp";
    //create gnuplot script
    std::ofstream gp(gp_file);

    //configure output image
    gp << "#!/usr/bin/env gnuplot\n";
    gp << "set terminal pngcairo size 800,400 background rgb '#1e1e1e'\n";
    gp << "set output '" << outDir << "/" << filename << ".png'\n";
    
    //configure plot appearance
    gp << "set border lc rgb '#e6e6e6'\n";
    gp << "set tics tc rgb '#e6e6e6'\n";
    gp << "set key tc rgb '#e6e6e6'\n";
    gp << "set grid lc rgb '#3a3a3a'\n";

    //configure title and axis labels
    gp << "set title '" << label << "' textcolor rgb '#e6e6e6'\n";
    gp << "set xlabel 'time (s)' textcolor rgb '#e6e6e6'\n";
    gp << "set ylabel '" << label << "' textcolor rgb '#e6e6e6'\n";
    
    //display bg grid
    gp << "set grid\n";

    //display the value versus time
    gp << "plot '" << path << "' every ::1 using 1:2 with lines lw 2 title '" << label << "'\n";
   
    //write gnuplot to disk
    gp.close();

    //execute the gnuplot script
    std::system(("gnuplot '" + escapePath(gp_file) + "'").c_str());
}

int main(int argc, char* argv[]) {
    //read cmd line args for sd card dir, output dir, and optionaly starting pose
    std::string sd_dir = (argc > 1) ? argv[1] : "";
    std::string out_dir = (argc > 2) ? argv[2] : "";
    double start_x = (argc > 3) ? std::stod(argv[3]) : 0.0;
    double start_y = (argc > 4) ? std::stod(argv[4]) : 0.0;

    //ensure both required directories were provided
    if (sd_dir.empty() || out_dir.empty()) {
        std::cout << "SD card path or output directory not specified in call\n";
        return 1;
    }

    //create output dir if it doesn't exist
    std::filesystem::create_directories(out_dir);

    //store the trajectory offset
    StartPose start{start_x, start_y};

    //find every set of related log files
    std::vector<LogSet> logs = findLogs(sd_dir);

    //generate gnuplots for each auton path run
    for (const auto& log: logs) {
        std::cout << "Processing: " << log.base_name << "\n";

        //generate trajectory dashboard (path + error)
        generateTrajectoryGNUPlot(log, out_dir, start);

        //generate combined plot for all value files
        generateAllValueFileGNUPlots(log, out_dir);

        //generate one plot for each value log
        for (const auto& value_file : log.value_files) {
            generateValueFileGNUPlot(value_file, out_dir);
        }
    }
    return 0;
}