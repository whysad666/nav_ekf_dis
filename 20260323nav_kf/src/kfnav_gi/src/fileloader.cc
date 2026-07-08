/*
 * OB_GINS: An Optimization-Based GNSS/INS Integrated Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Hailiang Tang
 *    Contact : thl@whu.edu.cn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "fileio/fileloader.h"

#include <cstddef>
#include <sstream>
#include <iostream>

FileLoader::FileLoader(const string &filename, size_t columns, int filetype) {
    open(filename, columns, filetype);
}

auto FileLoader::open(const string &filename, size_t columns, int filetype) -> bool {
    auto type = filetype == TEXT ? std::ios_base::in : (std::ios_base::in | std::ios_base::binary);
    filefp_.open(filename, type);

    columns_  = columns;
    filetype_ = filetype;
    return isOpen();
}

auto FileLoader::load(bool & flag) -> vector<double> {
    flag=load_();

    return data_;
}

auto FileLoader::loadn(int epochs) -> vector<vector<double>> {
    vector<vector<double>> datas;
    datas.clear();

    for (int k = 0; k < epochs; k++) {
        if (load_()) {
            datas.push_back(std::move(data_));
        } else {
            break;
        }
    }

    return datas;
}

auto FileLoader::load(vector<double> &data) -> bool {
    if (load_()) {
        data = std::move(data_);
        return true;
    }

    return false;
}

auto FileLoader::loadn(vector<vector<double>> &datas, int epochs) -> bool {
    datas.clear();

    for (int k = 0; k < epochs; k++) {
        if (load_()) {
            datas.push_back(std::move(data_));
        } else {
            break;
        }
    }

    return !datas.empty();
}

auto FileLoader::load_() -> bool {
    if (isEof())
        return false;

    data_.resize(columns_);

    if (filetype_ == TEXT) {
        string line;
        std::getline(filefp_, line);
        if (line.empty())
            return false;

        std::stringstream spl(line);

        data_.clear();
        for(size_t i=0;i<columns_;i++){
            double val;
            spl>>val;
            data_.push_back(val);
        }
    } else {
        filefp_.read((char *) data_.data(), sizeof(double) * columns_);
    }

    return true;
}
