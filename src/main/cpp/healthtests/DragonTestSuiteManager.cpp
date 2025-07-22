//====================================================================================================================================================
// Copyright 2025 Lake Orion Robotics FIRST Team 302
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

#include "healthtests/DragonTestSuiteManager.h"

DragonTestSuiteManager *DragonTestSuiteManager::m_instance = nullptr;

DragonTestSuiteManager *DragonTestSuiteManager::GetInstance()
{
    if (DragonTestSuiteManager::m_instance == nullptr)
    {
        DragonTestSuiteManager::m_instance = new DragonTestSuiteManager();
    }
    return DragonTestSuiteManager::m_instance;
}

DragonTestSuiteManager::DragonTestSuiteManager() : m_testSuites(),
                                                   m_currTestSlot(0),
                                                   m_currTest(nullptr),
                                                   m_testSuiteNames(),
                                                   m_currTestSuiteIndex(0)
{
}

// assumes we will run through all tests
void DragonTestSuiteManager::Init()
{
    // initialize from first test suite and case
    m_currTestSlot = 0;
    m_currTestSuiteIndex = 0;

    GetNextTest();
}

void DragonTestSuiteManager::RegisterTest(string testSuiteName, DragonTestCase *tc)
{
    // if test suite already exists, add test case
    if (m_testSuites.count(testSuiteName) > 0)
    {
        m_testSuites.at(testSuiteName).emplace_back(tc);
    }
    // otherwise make new vector
    else
    {
        m_testSuites.emplace(testSuiteName, std::vector<DragonTestCase *>{tc});
        m_testSuiteNames.emplace_back(testSuiteName);
    }
}

void DragonTestSuiteManager::Run()
{
    // return early if no test available
    if (m_currTest == nullptr)
        return;

    if (m_currTest->Run())
    {
        m_currTest->CompareAndReport();
        GetNextTest();
    }
}

void DragonTestSuiteManager::GetNextTest()
{
    // if all test suites have been iterated through, return early
    if (m_currTestSuiteIndex >= (int)m_testSuiteNames.size())
    {
        m_currTest = nullptr;
        return;
    }

    std::vector testCases = m_testSuites[m_testSuiteNames[m_currTestSuiteIndex]];
    if (m_currTestSlot >= (int)testCases.size())
    {
        m_currTestSlot = 0;
        m_currTestSuiteIndex++;

        if (m_currTestSuiteIndex >= (int)m_testSuiteNames.size())
        {
            m_currTest = nullptr;
            return;
        }

        testCases = m_testSuites[m_testSuiteNames[m_currTestSuiteIndex]];
    }

    m_currTest = testCases[m_currTestSlot];
    m_currTest->SetUp();
    m_currTestSlot++;
}