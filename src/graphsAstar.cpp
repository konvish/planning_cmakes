#define _CRT_SECURE_NO_WARNINGS
#include <mrpt/graphs/CAStarAlgorithm.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>

using namespace std;
using namespace mrpt::graphs;

class CCoinDistribution
{
    public:
    size_t coins2{0};
    size_t coins7{0};
    size_t coins8{0};
    size_t coins19{0};
    CCoinDistribution(){}
    size_t money() const
    {
        return 2 * coins2 + 7 * coins7 + 8 * coins8 + 19 * coins19;
    }

    inline bool operator==(const CCoinDistribution mon) const{
        return (coins2 == mon.coins2) && (coins7 == mon.coins7) && (coins8 == mon.coins8) && (coins19 == mon.coins19);
    }
};

class CAstarExample : public CAStarAlgorithm<CCoinDistribution>
{
    private:
    const size_t N;

    public:
    CAstarExample(size_t goal) : N(goal) {}
    bool isSolutionEnded(const CCoinDistribution& s) override
    {
        return s.money() == N;
    }

    bool isSolutionValid(const CCoinDistribution& s) override
    {
        return s.money() <= N;
    }

    void generateChildren(const CCoinDistribution& s,vector<CCoinDistribution>& sols) override
    {
        sols = vector<CCoinDistribution>(4,s);
        sols[0].coins2++;
        sols[1].coins7++;
        sols[2].coins8++;
        sols[3].coins19++;
    }

    double getHeuristic(const CCoinDistribution& s) override
    {
        return static_cast<double>(N - s.money()) / 19.0;
    }

    double getCost(const CCoinDistribution& s) override
    {
        return s.coins2 + s.coins7 + s.coins8 + s.coins19;
    }
};

int main(int argc,char** argv)
{
    for(;;)
    {
        char text[11];
        printf("Input an integer numver to solve a problem,or \"e\" to end.\n");
        if (1 != scanf("%10s",text))
        {
            printf("please,input a positive integer.\n\n");
            continue;
        }
        if (strlen(text) == 1 && (text[0] == 'e' || text[0] == 'E')) break;
        int val = atoi(text);
        if (val <= 0)
        {
            printf("Please,input a positive integer.\n\n");
            continue;
        }
        CCoinDistribution solIni,solFin;
        CAstarExample prob(static_cast<size_t>(val));
        switch (prob.getOptimalSolution(solIni,solFin,HUGE_VAL,15))
        {
        case 0:
            printf("No solution has been found.Either the number is too small, or the time elapsed has exceeded 15 seconds.\n\n");
            break;
        case 1:
            printf("An optimal solution has been found:\n");
            printf("\t%u coins of 2 piastres.\n\t%u coins of 7 piastres.\n\t%u coins of 8 piastres.\n\t%u coins of 19 piastres.\n\n",(unsigned)solFin.coins2,(unsigned)solFin.coins7,(unsigned)solFin.coins8,(unsigned)solFin.coins19);
            break;      
        case 2:
            printf("An solution has been found,although it may not be optimal:\n");
            printf("\t%u coins of 2 piastres.\n\t%u coins of 7 piastres.\n\t%u coins of 8 piastres.\n\t%u coins of 19 piastres.\n\n",(unsigned)solFin.coins2,(unsigned)solFin.coins7,(unsigned)solFin.coins8,(unsigned)solFin.coins19);
            break;      
        default:
            break;
        }
    }
    return 0;
}