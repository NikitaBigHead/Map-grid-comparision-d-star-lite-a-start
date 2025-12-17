# import sys
# import os

# # Добавляем корень проекта в путь
# project_root = os.path.dirname(os.path.abspath(__file__))
# sys.path.insert(0, project_root)

# from test_runner import run_comparison_tests
# from multi_robot.multi_robot_runner import run_multi_robot_comparison

# def main():
#     # print("🤖 A* vs D* Lite ALGORITHM COMPARISON")
#     # print("=" * 70)
#     # print("Part 1: Static maps with limited visibility")
#     # print("-" * 70)
    
#     # run_comparison_tests()
    
#     # print("\n" + "=" * 70)
#     # print("Part 2: Dynamic environment with moving obstacle")
#     # print("-" * 70)
    
#     run_multi_robot_comparison()
    
#     print("\n" + "=" * 70)
#     print("✅ All tests completed!")
#     print("Files saved in:")
#     print("  • robot_visualizations_blind/ - Static map tests")
#     print("  • multi_robot_visualizations/ - Dynamic obstacle test")

# if __name__ == "__main__":
#     main()

# Project/main.py
import sys
import os

# Добавляем корень проекта в путь (на самом деле, это не обязательно, но не вредит)
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

# from multi_robot.test_runner import run_comparison_tests  # Для статических тестов
from multi_robot.multi_robot_runner import run_multi_robot_comparison

def main():
    # print("🤖 A* vs D* Lite ALGORITHM COMPARISON")
    # print("=" * 70)
    # print("Part 1: Static maps with limited visibility")
    # print("-" * 70)
    
    # run_comparison_tests()
    
    # print("\n" + "=" * 70)
    # print("Part 2: Dynamic environment with moving obstacle")
    # print("-" * 70)
    
    run_multi_robot_comparison()
    
    print("\n" + "=" * 70)
    print("✅ All tests completed!")
    print("Files saved in:")
    print("  • robot_visualizations_blind/ - Static map tests")
    print("  • multi_robot_visualizations/ - Dynamic obstacle test")

if __name__ == "__main__":
    main()