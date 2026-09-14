"""통합 스마트팜 대시보드를 실행하는 PyQt 진입점이다."""

from __future__ import annotations

import sys

from PyQt5.QtWidgets import QApplication

from robot_tests.integrated.dashboard_app import IntegratedDashboard


# Qt 애플리케이션과 통합 대시보드 창을 생성하여 실행한다.
def main(args: list[str] | None = None) -> None:
    app = QApplication(sys.argv if args is None else args)
    window = IntegratedDashboard()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
