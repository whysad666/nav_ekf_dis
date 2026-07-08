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

#ifndef FILELOADER_H
#define FILELOADER_H

#include "filebase.h"

using std::vector;

class FileLoader : public FileBase {

public:
    FileLoader() = default;
    FileLoader(const string &filename, size_t columns, int filetype = TEXT);

    auto open(const string &filename, size_t columns, int filetype = TEXT) -> bool;

    auto load(bool & flag) -> vector<double>;
    auto loadn(int epochs) -> vector<vector<double>>;

    auto load(vector<double> &data) -> bool;
    auto loadn(vector<vector<double>> &data, int epochs) -> bool;

private:
    vector<double> data_;

    auto load_() -> bool;
};

#endif // FILELOADER_H
