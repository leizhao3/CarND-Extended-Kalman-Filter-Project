#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
//#include <bits/stdc++.h> 

using std::cout;
using std::endl;
using std::ifstream;
using std::ofstream;
using std::istringstream;
using std::string;
using std::vector;

int main() {
    const char *split_file_name_[3] = {"tools_2","FusionEKF","kalman_filter"};

    for(int i=0; i<3; i++){
        string file_name_ = split_file_name_[i];
        string in_file_name_ = file_name_ + ".h";
        string path = "split/";
        string out_file_name_h = path+file_name_+".h";
        string out_file_name_cpp = path+file_name_+".cpp";

        ifstream in_file(in_file_name_.c_str(), ifstream::in);
        if (!in_file.is_open()) {
            cout << "Cannot open input file: " << in_file_name_ << endl;
        }

        ofstream out_file_h(out_file_name_h, ofstream::out);
        if (!out_file_h.is_open()) {
            cout << "Cannot open out_file_h: " << out_file_name_h << endl;
        }

        ofstream out_file_cpp(out_file_name_cpp, ofstream::out);
        if (!out_file_cpp.is_open()) {
            cout << "Cannot open out_file_cpp: " << out_file_name_cpp << endl;
        }
        out_file_cpp << "#include \""<<in_file_name_<<"\"\n";

        cout << "Splitting "<<in_file_name_<<".....\n";
        int j = 0;
        string line;
        string dividing_line_ = "/*--------------------------start of cpp----------------------------------*/";

        
        while (getline(in_file, line)) {
            if (line.compare(dividing_line_) == 0){
                break;
            }
            out_file_h << line <<"\n";
        }

        while (getline(in_file, line)) {
            if (line.compare(0,6,"#endif") == 0){
                out_file_h << line <<"\n";
            }
            out_file_cpp << line <<"\n";
        }
        

        if (in_file.is_open()) {in_file.close();}
        if (out_file_h.is_open()) {out_file_h.close();}
        if (out_file_cpp.is_open()) {out_file_cpp.close();}

    }
    
    return 0;

}
