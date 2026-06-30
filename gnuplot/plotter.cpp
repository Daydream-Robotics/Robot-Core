#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <map>

struct LogSet {
    std::string base_name;
    std::string target_pos_file;
    std::string actual_pos_file;
    std::string error_pos_file;
    std::vector<std::string> value_files;
};

struct StartPose {
    double x = 0;
    double y = 0;
};


std::string generate_recentered_file(const std::string& original_file, const std::string& outDir, const std::string& suffix, double offsetX, double offsetY, bool isTarget) {
    if (original_file.empty() || !std::filesystem::exists(original_file)) {
        return "";
    }
    std::string recentered_name = outDir + "/temp_" + suffix + ".dat";
    std::ofstream out(recentered_name);
    std::ifstream in(original_file);

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        double t, x, y;
        if (isTarget) {
            if (sscanf(line.c_str(), "%lf %lf %lf", &t, &x, &y) == 3) {
                out << t << " " << (x + offsetX) << " " << (y + offsetY) << "\n";
            }
        }
    }

    out.close();
    return recentered_name;
}


std::string escapePath(const std::string& path) {
    std::string result;
    for (char c : path) {
        if (c == '\'') result += "\\'";
        else result += c;
    }
    return result;
}

std::vector<LogSet> findLogs(const std::string& dir) {
    std::vector<LogSet> sets;
    std::map<std::string, LogSet> groups;

    for (const auto& entry : std::filesystem::directory_iterator(dir)){
        std::string filename = entry.path().filename().string();
        size_t dot_pos = filename.find_last_of('.');
        if (dot_pos == std::string::npos || filename.substr(dot_pos) != ".dat") {
            continue;
        }

        size_t underscore_pos = filename.find_last_of('_');
        if (underscore_pos == std::string::npos) {
            continue;
        }
        std::string basename = filename.substr(0, underscore_pos);
        std::string type = filename.substr(underscore_pos + 1, dot_pos - underscore_pos -1);

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
            groups[basename].value_files.push_back(entry.path().string());
        }
        groups[basename].base_name = basename;
    }

    for (auto& [base, set] : groups) {
        sets.push_back(set);
    }

    return sets;
}

void generateGNUPlotFiles (const LogSet& log, const std::string& outDir, const StartPose& start) {
    std::string temp_target = generate_recentered_file(log.target_pos_file, outDir, "target", start.x, start.y, true);
    std::string temp_actual = generate_recentered_file(log.actual_pos_file, outDir, "actual", start.x, start.y, true);


    std::string gp_file = outDir + "/plot_" + log.base_name + ".gp";

    std::ofstream gp(gp_file);
    gp << "#!/usr/bin/env gnuplot\n";
    gp << "set terminal pngcairo size 1600,800 enhanced font 'Arial,12' background rgb '#1e1e1e'\n";
    gp << "set output '" << outDir << "/" << log.base_name << "_dashboard.png'\n";
    gp << "set border lc rgb '#e6e6e6'\n";
    gp << "set tics tc rgb '#e6e6e6'\n";
    gp << "set key tc rgb '#e6e6e6'\n";
    gp << "set grid lc rgb '#3a3a3a'\n";
    gp << "set multiplot layout 1,2 title '" << log.base_name << "' textcolor rgb '#e6e6e6'\n\n";

    gp << "set size ratio 1\n";
    gp << "set xrange [-72:72]\n";
    gp << "set yrange [-72:72]\n";
    gp << "set xtics 24\n";
    gp << "set ytics 24\n";
    gp << "set mxtics 2\n";
    gp << "set mytics 2\n";
    gp << "set title 'Path' textcolor rgb '#e6e6e6'\n";
    gp << "set xlabel 'x (inches)' textcolor rgb '#e6e6e6'\n";
    gp << "set ylabel 'y (inches)' textcolor rgb '#e6e6e6'\n";
    gp << "set grid xtics ytics mxtics mytics\n";
    gp << "set key top left\n";

    gp << "set zeroaxis lc rgb '#666666' lw 1\n";
    gp << "set object 1 rectangle from -72,-72 to 72,72 fillstyle empty border lc rgb '#e6e6e6' lw 2\n";

    gp << "plot ";

    bool first = true;
    auto add = [&](const std::string& file_path, const std::string& using_cols, const std::string& style, const std::string& title, const std::string& color) {
        if (file_path.empty() || !std::filesystem::exists(file_path)) {
            return;
        }
        if (!first) {
            gp << ", \\\n     ";
        }
        first = false;
        gp << "'" << escapePath(file_path) << "' using " << using_cols << " " << style << " lc rgb '" << color << "' title '" << title << "'";
    };

    add(temp_target, "2:3", "with lines lw 2", "target", "#4a9eff");
    add(temp_actual, "2:3", "with lines lw 1", "actual", "#5cdb5c");

    gp << "\n\n";

    gp << "set size ratio 0\n";
    gp << "set autoscale x\n";
    gp << "set yrange [0:6]\n";
    gp << "set ytics 1\n";
    gp << "set mytics 2\n";
    gp << "unset object 1\n";
    gp << "set title 'Position Error' textcolor rgb '#e6e6e6'\n";
    gp << "set xlabel 'time (s)' textcolor rgb '#e6e6e6'\n";
    gp << "set ylabel 'error (inches)' textcolor rgb '#e6e6e6'\n";
    gp << "set grid\n";
    gp << "set key top left\n";

    gp << "plot '" << escapePath(log.error_pos_file) << "' using 1:2 with filledcurves y1=0 lc rgb '#5a2222' title '', \\\n";
    gp << "     '" << escapePath(log.error_pos_file) << "' using 1:2 with lines lw 2 lc rgb '#ff6b6b' title 'error'\n";

    gp << "unset multiplot\n";
    gp.close();

    std::system(("gnuplot '" + escapePath(gp_file) + "'").c_str());
    std::cout << "Generated: " << outDir << "/" << log.base_name << "_dashboard.png\n";
    if (!temp_target.empty()) {
        std::filesystem::remove(temp_target);
    }
    if (!temp_actual.empty()) {
        std::filesystem::remove(temp_actual);
    }
}

void generateValuesPlot(const LogSet& log, const std::string& outDir) {
    if (log.value_files.empty()) {
        return;
    }
    std::string gp_file = outDir + "/plot_" + log.base_name + "_values.gp";

    std::ofstream gp(gp_file);
    gp << "#!/usr/bin/env gnuplot\n";
    gp << "set terminal pngcairo size 1600,800 enhanced font 'Arial,12' background rgb '#1e1e1e'\n";
    gp << "set output '" << outDir << "/" << log.base_name << "_values.png'\n";
    gp << "set border lc rgb '#e6e6e6'\n";
    gp << "set tics tc rgb '#e6e6e6'\n";
    gp << "set key tc rgb '#e6e6e6'\n";
    gp << "set grid lc rgb '#3a3a3a'\n";
    gp << "set title '" << log.base_name << " - Value Plots' textcolor rgb '#e6e6e6'\n";
    gp << "set xlabel 'time (s)' textcolor rgb '#e6e6e6'\n";
    gp << "set ylabel 'value' textcolor rgb '#e6e6e6'\n";
    gp << "set grid\n";
    gp << "set key top left\n";

    gp << "plot ";
    bool first = true;

    for (const std::string& value_file : log.value_files) {
        std::string stem = std::filesystem::path(value_file).stem().string();
        size_t underscore_pos = stem.find_last_of('_');
        std::string label = (underscore_pos != std::string::npos) ? stem.substr(underscore_pos + 1) : stem;

        if(!first) {
            gp << ", \\\n     ";
        }
        first = false;
        gp << "'" << escapePath(value_file) << "' every ::1 using 1:2 with lines lw 2 title '" << label << "'";
    }

    gp << "\n";
    gp.close();

    std::system(("gnuplot '" + escapePath(gp_file) + "'").c_str());
    std::cout << "Generated: " << outDir << "/" << log.base_name << "_values.png\n";
}


void generateValuePlots(const std::string& path, const std::string& outDir) {
    std::string filename = std::filesystem::path(path).stem().string();
    size_t underscore_pos = filename.find_last_of('_');
    std::string label = (underscore_pos != std::string::npos) ? filename.substr(underscore_pos + 1) : "value";

    std::string gp_file = outDir + "/plot_" + filename + ".gp";

    std::ofstream gp(gp_file);
    gp << "#!/usr/bin/env gnuplot\n";
    gp << "set terminal pngcairo size 800,400 background rgb '#1e1e1e'\n";
    gp << "set output '" << outDir << "/" << filename << ".png'\n";
    gp << "set border lc rgb '#e6e6e6'\n";
    gp << "set tics tc rgb '#e6e6e6'\n";
    gp << "set key tc rgb '#e6e6e6'\n";
    gp << "set grid lc rgb '#3a3a3a'\n";
    gp << "set title '" << label << "' textcolor rgb '#e6e6e6'\n";
    gp << "set xlabel 'time (s)' textcolor rgb '#e6e6e6'\n";
    gp << "set ylabel '" << label << "' textcolor rgb '#e6e6e6'\n";
    gp << "set grid\n";
    gp << "plot '" << path << "' every ::1 using 1:2 with lines lw 2 title '" << label << "'\n";
    gp.close();

    std::system(("gnuplot '" + escapePath(gp_file) + "'").c_str());
}

int main(int argc, char* argv[]) {
    std::string sd_dir = (argc > 1) ? argv[1] : "";
    std::string out_dir = (argc > 2) ? argv[2] : "";
    double start_x = (argc > 3) ? std::stod(argv[3]) : 0.0;
    double start_y = (argc > 4) ? std::stod(argv[4]) : 0.0;
    if (sd_dir.empty() || out_dir.empty()) {
        std::cout << "SD card path or output directory not specified in call\n";
        return 1;
    }

    std::filesystem::create_directories(out_dir);

    StartPose start{start_x, start_y};

    std::vector<LogSet> logs = findLogs(sd_dir);
    for (const auto& log: logs) {
        std::cout << "Processing: " << log.base_name << "\n";
        generateGNUPlotFiles(log, out_dir, start);
        generateValuesPlot(log, out_dir);
        for (const auto& value_file : log.value_files) {
            generateValuePlots(value_file, out_dir);
        }
    }
    return 0;
}