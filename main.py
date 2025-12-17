# main.py
from test_runner import run_comparison_tests
from multi_robot.multi_robot_runner import run_multi_robot_comparison

def main():
    print("🤖 Сравнение A* и D* Lite в навигации роботов")
    print("=" * 70)
    
    print("\nЧасть 1: Один робот с ограниченным обзором (радиус 10 клеток)")
    print("-" * 70)
    run_comparison_tests()
    
    print("\nЧасть 2: Два робота — динамическое препятствие (конфликт)")
    print("-" * 70)
    run_multi_robot_comparison()
    
    print("\n✅ Все тесты завершены!")
    print("Результаты:")
    print("   • Анимации одного робота: robot_visualizations_blind/")
    print("   • Анимация двух роботов: multi_robot_visualizations/")

if __name__ == "__main__":
    main()