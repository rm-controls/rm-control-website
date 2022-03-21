import React from "react";
import BrowserOnly from "@docusaurus/BrowserOnly";
import Container from "./../../components/container/container";
import WhyContext from "../../../../current_docs/overview/why_rm-controls.md";
import EnWhyContext from "../../../../i18n/en/docusaurus-plugin-content-docs/current/overview/why_rm-controls.md";

export default function Why() {
  const component = "shift-why";

  return (
    <div className={component}>
      <Container componentClass={component} size={"medium"}>
        <BrowserOnly>
          {() =>
            window.location.href.indexOf("en/") != -1 ? (
              <EnWhyContext />
            ) : (
              <WhyContext />
            )
          }
        </BrowserOnly>
      </Container>
    </div>
  );
}
