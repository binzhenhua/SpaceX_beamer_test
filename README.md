this is an exmaple how to compile and run the test, use a python script to evaluate the result.
g++ -O3 -std=c++17 .\startlink_beam_planner_Q3_myselft.cpp -o mybeamer

.\mybeamer.exe .\test_cases\11_one_hundred_thousand_users.txt | python ./evaluate.py .\test_cases\11_one_hundred_thousand_users.txt
Reading scenario file .\test_cases\11_one_hundred_thousand_users.txt
Reading solution from stdin.
Checking user coverage...
29.237999999999996% of 100000 total users covered.
Checking each user can see their assigned satellite...
        All users' assigned satellites are visible.
Checking no sat interferes with itself...
        No satellite self-interferes.
Checking no sat interferes with a non-Starlink satellite...
        No satellite interferes with a non-Starlink satellite!

Solution passed all checks!



You can implement your solution and challenge this problem.

The problem description and some current SpaceX CDN research are in the PDF document.
