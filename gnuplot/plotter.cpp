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

void generateGNUPlotFiles (const LogSet& log, const std::string& outDir) {
    std::string gp_file = outDir + "/plot_" + log.base_name + ".gp";

    std::ofstream gp(gp_file);
    gp << "#!/usr/bin/env gnuplot\n";
    gp << "set terminal pngcairo size 1400,600 enhanced font 'Arial,12'\n";
    gp << "set output '" << outDir << "/" << log.base_name << "_dashboard.png'\n";
    gp << "set multiplot layout 1,2 title '" << log.base_name << "'\n\n";

    gp << "set size ratio 1\n";
    gp << "set xrange [-72:72]\n";
    gp << "set yrange [-72:72]\n";
    gp << "set xtics 24\n";
    gp << "set ytics 24\n";
    gp << "set title 'Path'\n";
    gp << "set xlabel 'x (inches)'\n";
    gp << "set ylabel 'y (inches)'\n";
    gp << "set grid\n";
    gp << "set key top left\n";
    gp << "set object 1 rectangle from -72,-72 to 72,72 fillstyle empty border lc rgb 'black' lw 2\n";

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
        gp << "'" << file_path << "' using " << using_cols << " " << style << " lc rgb '" << color << "' title '" << title << "'";
    };

    add(log.target_pos_file, "2:3", "with lines lw 2", "target", "blue");
    add(log.actual_pos_file, "2:3", "with lines lw 1", "target", "green");

    gp << "set size ratio 0\n";
    gp << "set autoscale\n";
    gp << "unset object 1\n";
    gp << "set title 'Error & Values'\n";
    gp << "set xlabel 'time (s)'\n";
    gp << "set ylabel 'value'\n";
    gp << "set grid\n";
    gp << "set key top left\n";

    gp << "plot ";
    first = true;

    add(log.error_pos_file, "1:2", "with lines lw 2", "error", "red");

    for (const auto& value_file: log.value_files) {
        std::string label = std::filesystem::path(value_file).stem().string();
        size_t underscore_pos = label.find_last_of('_');
        if (underscore_pos != std::string::npos) {
            label = label.substr(underscore_pos + 1);
        }

        if (!first) {
            gp << ", \\\n     ";
        }
        first = false;
        gp << "'" << value_file << "' every ::1 using 1:2 with lines lw 1 title '" << label << "'";
    }

    if (first) {
        gp << "0 notitle";
    }
    gp << "\nunset multiplot\n";
    gp.close();

    std::system(("gnuplot '" + gp_file + "'").c_str());
    std::cout << "Generated: " << outDir << "/" << log.base_name << "_dashboard.png\n";
}

void generateValuePlot(const std::string& path, const std::string& outDir) {
    std::string filename = std::filesystem::path(path).stem().string();
    size_t underscore_pos = filename.find_last_of('_');
    std::string label = (underscore_pos != std::string::npos) ? filename.substr(underscore_pos + 1) : "value";

    std::string gp_file = outDir + "/plot_" + filename + ".gp";

    std::ofstream gp(gp_file);
    gp << "#!/usr/bin/env gnuplot\n";
    gp << "set terminal pngcairo size 800,400\n";
    gp << "set output '" << outDir << "/" << filename << ".png'\n";
    gp << "set title '" << label << "'\n";
    gp << "set xlabel 'time (s)'\n";
    gp << "set ylabel '" << label << "'\n";
    gp << "set grid\n";
    gp << "plot '" << path << "' every ::1 using 1:2 with lines lw 2 title '" << label << "'\n";
    gp.close();

    std::system(("gnuplot '" + gp_file + "'").c_str());
}

int main(int argc, char* argv[]) {
    std::string sd_dir = (argc > 1) ? argv[1] : "";
    std::string out_dir = (argc > 2) ? argv[2] : "";
    if (sd_dir.empty() || out_dir.empty()) {
        std::cout << "SD card path or output directory not specified in call\n";
        return 1;
    }

    std::filesystem::create_directories(out_dir);

    std::vector<LogSet> logs = findLogs(sd_dir);
    for (const auto& log: logs) {
        std::cout << "Processing: " << log.base_name << "\n";
        generateGNUPlotFiles(log, out_dir);

        for (const auto& value_file : log.value_files) {
            generateValuePlot(value_file, out_dir);
        }
    }
    return 0;
}