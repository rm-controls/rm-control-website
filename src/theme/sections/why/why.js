import React, { useState } from "react";
import Container from "./../../components/container/container";
import WhyContext from "../../../../current_docs/overview/why_rm-controls.md";
import EnWhyContext from "../../../../i18n/en/docusaurus-plugin-content-docs/current/overview/why_rm-controls.md";

export default function Why() {
  const component = "shift-why";

  const [langCode, setLangCode] = useState(
    window.location.href.indexOf("en/") != -1 ? "en" : "zh"
  );

  return (
    <div className={component}>
      <Container componentClass={component} size={"medium"}>
        {langCode == "en" ? <EnWhyContext /> : <WhyContext />}
      </Container>
    </div>
  );
}
